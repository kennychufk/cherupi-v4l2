#include "websocket_server.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string_view>

using json = nlohmann::json;

WebSocketServer::WebSocketServer() {
  camera_manager = std::make_unique<CameraManager>();
  frame_saver = std::make_unique<FrameSaver>();
  stream_manager =
      std::make_unique<StreamManager>(camera_manager.get(), frame_saver.get());

  // Set up the callback for frame notifications
  camera_manager->setStreamManagerNotify([this]() {
    if (stream_manager) {
      stream_manager->notifyFrameAvailable();
    }
  });
}

WebSocketServer::~WebSocketServer() {
  LOG_INFO("WebSocketServer", "Shutting down server");
  stop();
}

void WebSocketServer::run() {
  LOG_INFO("WebSocketServer",
           "Starting WebSocket server on port " + std::to_string(PORT));

  app = new uWS::App();

  app->ws<int>(
         "/*",
         {// Settings
          .compression = uWS::DISABLED,
          .maxPayloadLength = 16 * 1024 * 1024,  // 16MB max
          .idleTimeout = 120,
          .maxBackpressure = 1024 * 1024 * 10,  // 10MB

          // Handlers
          .upgrade =
              [this](auto* res, auto* req, auto* context) {
                // Block if we already have a client
                if (has_client.load()) {
                  LOG_WARN(
                      "WebSocketServer",
                      "Rejecting connection - server already has a client");
                  res->writeStatus("503 Service Unavailable");
                  res->writeHeader("Content-Type", "text/plain");
                  res->end("Server already has an active connection");
                  return;
                }

                // Accept the connection
                res->template upgrade<int>(
                    {}, req->getHeader("sec-websocket-key"),
                    req->getHeader("sec-websocket-protocol"),
                    req->getHeader("sec-websocket-extensions"), context);
              },

          .open =
              [this](auto* ws) {
                LOG_INFO("WebSocketServer",
                         "Client connected from " +
                             std::string(ws->getRemoteAddressAsText()));
                has_client = true;
                stream_manager->setWebSocket(ws);
              },

          .message =
              [this](auto* ws, std::string_view message, uWS::OpCode opCode) {
                if (opCode != uWS::OpCode::TEXT) {
                  sendError(ws, "Only text messages are accepted for commands");
                  return;
                }

                try {
                  json msg = json::parse(message);
                  std::string cmd = msg["cmd"];

                  LOG_DEBUG("WebSocketServer", "Received command: " + cmd);

                  if (cmd == Protocol::CMD_DISCOVER) {
                    handleDiscover(ws);
                  } else if (cmd == Protocol::CMD_CONFIGURE) {
                    handleConfigure(ws, msg.value("params", json::object()));
                  } else if (cmd == Protocol::CMD_SET_SAVE_MODE) {
                    handleSetSaveMode(ws, msg);
                  } else if (cmd == Protocol::CMD_START_CAMERAS) {
                    handleStartCameras(ws);
                  } else if (cmd == Protocol::CMD_START_STREAM) {
                    handleStartStream(ws, msg);
                  } else if (cmd == Protocol::CMD_STOP_STREAM) {
                    handleStopStream(ws, msg);
                  } else if (cmd == Protocol::CMD_STOP_CAMERAS) {
                    handleStopCameras(ws);
                  } else if (cmd == Protocol::CMD_RESET_FRAME_COUNTS) {
                    handleResetFrameCounts(ws);
                  } else if (cmd == Protocol::CMD_SET_HEADER_ONLY) {
                    handleSetHeaderOnly(ws, msg);
                  } else {
                    sendError(ws, "Unknown command: " + cmd);
                  }
                } catch (json::exception& e) {
                  LOG_ERROR("WebSocketServer",
                            "JSON parse error: " + std::string(e.what()));
                  sendError(ws, std::string("JSON parse error: ") + e.what());
                } catch (std::exception& e) {
                  LOG_ERROR("WebSocketServer",
                            "Command handler error: " + std::string(e.what()));
                  sendError(ws, std::string("Error: ") + e.what());
                }
              },

          .drain =
              [this](auto* ws) {
                // Handle backpressure with hysteresis to prevent oscillation
                auto bufferedAmount = ws->getBufferedAmount();

                // Use different thresholds for entering and leaving
                // backpressure This prevents rapid oscillation between states
                static constexpr size_t BACKPRESSURE_HIGH_WATER =
                    512 * 1024;  // 512KB
                static constexpr size_t BACKPRESSURE_LOW_WATER =
                    128 * 1024;  // 128KB

                // Track backpressure state per connection (using static for
                // simplicity) In production, this should be stored per
                // WebSocket connection
                static bool wasInBackpressure = false;
                bool hasPressure = false;

                if (!wasInBackpressure) {
                  // Not in backpressure - use high water mark to enter
                  hasPressure = bufferedAmount > BACKPRESSURE_HIGH_WATER;
                } else {
                  // Already in backpressure - use low water mark to exit
                  // (hysteresis)
                  hasPressure = bufferedAmount > BACKPRESSURE_LOW_WATER;
                }

                // Only log and notify on state changes
                if (hasPressure != wasInBackpressure) {
                  if (hasPressure) {
                    LOG_DEBUG("WebSocketServer",
                              "Entering backpressure state: " +
                                  std::to_string(bufferedAmount) +
                                  " bytes buffered (threshold: " +
                                  std::to_string(BACKPRESSURE_HIGH_WATER) +
                                  ")");
                  } else {
                    LOG_DEBUG("WebSocketServer",
                              "Exiting backpressure state: " +
                                  std::to_string(bufferedAmount) +
                                  " bytes buffered (threshold: " +
                                  std::to_string(BACKPRESSURE_LOW_WATER) + ")");
                  }
                  wasInBackpressure = hasPressure;

                  // Notify stream manager of backpressure state change
                  stream_manager->notifyBackpressure(hasPressure);
                }

                // Optional: Log periodic status if buffer is very high
                if (bufferedAmount > 1024 * 1024) {  // 1MB
                  static auto lastHighBufferLog =
                      std::chrono::steady_clock::now();
                  auto now = std::chrono::steady_clock::now();
                  if (now - lastHighBufferLog > std::chrono::seconds(1)) {
                    LOG_WARN("WebSocketServer",
                             "High buffer warning: " +
                                 std::to_string(bufferedAmount / 1024) +
                                 " KB buffered");
                    lastHighBufferLog = now;
                  }
                }
              },

          .close =
              [this](auto*, int code, std::string_view message) {
                LOG_INFO("WebSocketServer",
                         "Client disconnected with code " +
                             std::to_string(code) +
                             (message.empty()
                                  ? ""
                                  : ", message: " + std::string(message)));

                // Log specific close codes
                switch (code) {
                  case 1000:
                    LOG_INFO("WebSocketServer", "Normal closure");
                    break;
                  case 1001:
                    LOG_INFO("WebSocketServer", "Going away");
                    break;
                  case 1006:
                    LOG_WARN("WebSocketServer",
                             "Abnormal closure - no close frame received");
                    break;
                  case 1009:
                    LOG_ERROR("WebSocketServer", "Message too big");
                    break;
                  default:
                    LOG_WARN("WebSocketServer",
                             "Unexpected close code: " + std::to_string(code));
                }

                cleanupConnection();
              }})
      .listen(PORT, [this](auto* listen_socket) {
        if (listen_socket) {
          LOG_INFO("WebSocketServer",
                   "Server listening on port " + std::to_string(PORT));
        } else {
          LOG_ERROR("WebSocketServer",
                    "Failed to listen on port " + std::to_string(PORT));
        }
      });

  // Run the event loop
  loop = uWS::Loop::get();
  app->run();
}

void WebSocketServer::stop() {
  LOG_INFO("WebSocketServer", "Stopping server");

  // Clean up any active connection
  cleanupConnection();

  if (app) {
    app->close();
    delete app;
    app = nullptr;
  }
}

void WebSocketServer::handleDiscover(uWS::WebSocket<false, true, int>* ws) {
  size_t count = camera_manager->discoverCameras();

  json response;
  response["type"] = Protocol::TYPE_DISCOVERY;
  response["cameras"] = json::array();

  for (size_t i = 0; i < count; i++) {
    json cam_info;
    cam_info["id"] = i;
    cam_info["type"] = "IMX519";
    response["cameras"].push_back(cam_info);
  }

  LOG_INFO("WebSocketServer",
           "Discovered " + std::to_string(count) + " cameras");
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleConfigure(uWS::WebSocket<false, true, int>* ws,
                                      const json& params) {
  if (system_state != CameraState::IDLE) {
    sendError(ws, "Cameras must be idle to configure");
    return;
  }

  CameraConfig config;

  // Override with any provided parameters
  if (params.contains("width")) config.width = params["width"];
  if (params.contains("height")) config.height = params["height"];
  if (params.contains("crop_width")) config.crop_width = params["crop_width"];
  if (params.contains("crop_height"))
    config.crop_height = params["crop_height"];
  if (params.contains("crop_left")) config.crop_left = params["crop_left"];
  if (params.contains("crop_top")) config.crop_top = params["crop_top"];

  // Optional AWB sub-config
  if (params.contains("awb") && params["awb"].is_object()) {
    const json& a = params["awb"];
    if (a.contains("enabled")) config.awb.enabled = a["enabled"];
    if (a.contains("interval")) config.awb.interval = a["interval"];
    if (a.contains("speed")) config.awb.speed = a["speed"];
    if (a.contains("warmup_frames"))
      config.awb.warmup_frames = a["warmup_frames"];
  }

  LOG_INFO("WebSocketServer", "Configuring cameras with resolution " +
                                  std::to_string(config.width) + "x" +
                                  std::to_string(config.height));

  if (camera_manager->configureAll(config)) {
    system_state = CameraState::CONFIGURED;
    sendStatus(ws, "All cameras configured successfully");
  } else {
    sendError(ws, "Failed to configure cameras");
  }
}

void WebSocketServer::handleSetSaveMode(uWS::WebSocket<false, true, int>* ws,
                                        const json& msg) {
  SaveConfig config;
  std::string mode = msg["mode"];

  if (mode == "none") {
    config.mode = SaveMode::NONE;
  } else if (mode == "buffer") {
    config.mode = SaveMode::BUFFER;
  } else if (mode == "batch") {
    config.mode = SaveMode::BATCH;
  } else if (mode == "checkerboard") {
    config.mode = SaveMode::CHECKERBOARD;
  } else {
    sendError(ws, "Invalid save mode: " + mode);
    return;
  }

  // Get optional parameters
  if (msg.contains("params")) {
    const json& params = msg["params"];
    if (params.contains("output_dir")) config.output_dir = params["output_dir"];
    if (params.contains("prepend_timestamp_to_dir"))
      config.prepend_timestamp_to_dir = params["prepend_timestamp_to_dir"];
    if (params.contains("batch_size")) config.batch_size = params["batch_size"];
    if (params.contains("writer_threads"))
      config.writer_threads = params["writer_threads"];

    // Checkerboard-specific parameters
    if (params.contains("checkerboard_rows"))
      config.checkerboard_rows = params["checkerboard_rows"];
    if (params.contains("checkerboard_cols"))
      config.checkerboard_cols = params["checkerboard_cols"];
    if (params.contains("checkerboard_full_res_detection"))
      config.checkerboard_full_res_detection =
          params["checkerboard_full_res_detection"];
    if (params.contains("checkerboard_num_threads"))
      config.checkerboard_num_threads = params["checkerboard_num_threads"];
  }

  frame_saver->configure(config);
  LOG_INFO("WebSocketServer", "Save mode configured: " + mode);
  sendStatus(ws, "Save mode configured: " + mode);
}

void WebSocketServer::handleStartCameras(uWS::WebSocket<false, true, int>* ws) {
  if (system_state != CameraState::CONFIGURED) {
    sendError(ws, "Cameras must be configured before starting");
    return;
  }

  // Start frame saver first (if not NONE mode)
  frame_saver->start();

  // Set up frame callback only if saving is enabled
  if (frame_saver->getMode() != SaveMode::NONE) {
    LOG_INFO("WebSocketServer", "Frame saving enabled, setting up callback");
    camera_manager->setFrameCallback(
        [this](const FrameData& frame) { frame_saver->saveFrame(frame); });
  } else {
    LOG_INFO("WebSocketServer", "Frame saving disabled, no callback set");
    camera_manager->setFrameCallback(nullptr);
  }

  // Start all cameras
  if (camera_manager->startAll()) {
    system_state = CameraState::RUNNING;
    sendStatus(ws, "All cameras started successfully");
  } else {
    frame_saver->stop();
    sendError(ws, "Failed to start cameras");
  }
}

void WebSocketServer::handleStartStream(uWS::WebSocket<false, true, int>* ws,
                                        const json& msg) {
  if (system_state != CameraState::RUNNING) {
    sendError(ws, "Cameras must be running to start streaming");
    return;
  }

  uint32_t camera_id = msg["camera_id"];

  if (stream_manager->startStreamingCamera(camera_id)) {
    LOG_INFO("WebSocketServer",
             "Started streaming camera " + std::to_string(camera_id));
    sendStatus(ws, "Started streaming camera " + std::to_string(camera_id));
  } else {
    sendError(ws,
              "Failed to start streaming camera " + std::to_string(camera_id));
  }
}

void WebSocketServer::handleStopStream(uWS::WebSocket<false, true, int>* ws,
                                       const json& msg) {
  uint32_t camera_id = msg["camera_id"];

  if (stream_manager->stopStreamingCamera(camera_id)) {
    LOG_INFO("WebSocketServer",
             "Stopped streaming camera " + std::to_string(camera_id));
    sendStatus(ws, "Stopped streaming camera " + std::to_string(camera_id));
  } else {
    sendError(ws, "Camera " + std::to_string(camera_id) + " was not streaming");
  }
}

void WebSocketServer::handleStopCameras(uWS::WebSocket<false, true, int>* ws) {
  if (system_state != CameraState::RUNNING) {
    sendError(ws, "Cameras are not running");
    return;
  }

  LOG_INFO("WebSocketServer", "Stopping all cameras");

  // Stop streaming first
  stream_manager->stopAllStreaming();

  // Stop all cameras
  camera_manager->stopAll();

  // Flush buffered frames if in buffer mode
  if (frame_saver->getMode() == SaveMode::BUFFER) {
    LOG_INFO("WebSocketServer", "Flushing buffered frames");
    frame_saver->flushBufferedFrames();
  }

  // Stop frame saver
  frame_saver->stop();

  system_state = CameraState::CONFIGURED;

  json status;
  status["type"] = Protocol::TYPE_STATUS;
  status["message"] = "All cameras stopped";
  status["frames_saved"] = frame_saver->getFramesSaved();
  status["bytes_written"] = frame_saver->getBytesWritten();

  ws->send(status.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleResetFrameCounts(
    uWS::WebSocket<false, true, int>* ws) {
  LOG_INFO("WebSocketServer", "Resetting frame counts for all cameras");

  camera_manager->resetFrameCounts();

  // Also reset frame saver counts
  if (frame_saver) {
    frame_saver->resetFramesSavedCounts();
  }

  sendStatus(ws, "Frame counts reset for all cameras");
}

void WebSocketServer::handleSetHeaderOnly(uWS::WebSocket<false, true, int>* ws,
                                          const json& msg) {
  bool enabled = msg.value("enabled", false);

  LOG_INFO("WebSocketServer",
           "Setting header only mode to " +
               std::string(enabled ? "enabled" : "disabled"));

  // Set the header only mode in the stream manager
  stream_manager->setHeaderOnlyMode(enabled);

  // Send confirmation status
  sendStatus(
      ws, "Header only mode " + std::string(enabled ? "enabled" : "disabled"));
}

void WebSocketServer::sendStatus(uWS::WebSocket<false, true, int>* ws,
                                 const std::string& message) {
  json response;
  response["type"] = Protocol::TYPE_STATUS;
  response["message"] = message;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::sendError(uWS::WebSocket<false, true, int>* ws,
                                const std::string& message) {
  LOG_ERROR("WebSocketServer", "Sending error: " + message);
  json response;
  response["type"] = Protocol::TYPE_ERROR;
  response["message"] = message;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::cleanupConnection() {
  LOG_INFO("WebSocketServer", "Cleaning up connection");

  // Stop streaming but keep cameras running
  stream_manager->clearWebSocket();

  // Note: We do NOT stop cameras here to allow frame saving to continue
  // Cameras will only be stopped by explicit stop_cameras command

  // Log current state
  if (system_state == CameraState::RUNNING) {
    LOG_INFO("WebSocketServer",
             "Cameras still running for potential frame saving");
  }

  // Reset client flag
  has_client = false;
}
