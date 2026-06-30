#include "websocket_server.hpp"

#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string_view>

#include "command_parser.hpp"

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
                active_sink = std::make_shared<UwsFrameSink>(ws);
                stream_manager->setSink(active_sink.get());
              },

          .message =
              [this](auto* ws, std::string_view message, uWS::OpCode opCode) {
                if (opCode != uWS::OpCode::TEXT) {
                  sendError(ws, "Only text messages are accepted for commands");
                  return;
                }

                auto parsed = command_parser::parseCommand(message);
                if (std::holds_alternative<std::string>(parsed)) {
                  const auto& err = std::get<std::string>(parsed);
                  LOG_ERROR("WebSocketServer", err);
                  sendError(ws, err);
                  return;
                }
                const auto& cmd = std::get<command_parser::ParsedCommand>(parsed);
                LOG_DEBUG("WebSocketServer",
                          "Received command: " +
                              cmd.message.value("cmd", std::string()));
                try {
                  switch (cmd.kind) {
                    case command_parser::CommandKind::Discover:
                      handleDiscover(ws);
                      break;
                    case command_parser::CommandKind::GetState:
                      handleGetState(ws);
                      break;
                    case command_parser::CommandKind::Configure:
                      handleConfigure(
                          ws, cmd.message.value("params", json::object()));
                      break;
                    case command_parser::CommandKind::Unconfigure:
                      handleUnconfigure(ws);
                      break;
                    case command_parser::CommandKind::SetSaveMode:
                      handleSetSaveMode(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::StartCameras:
                      handleStartCameras(ws);
                      break;
                    case command_parser::CommandKind::StartStream:
                      handleStartStream(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::StopStream:
                      handleStopStream(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::StopCameras:
                      handleStopCameras(ws);
                      break;
                    case command_parser::CommandKind::ResetFrameCounts:
                      handleResetFrameCounts(ws);
                      break;
                    case command_parser::CommandKind::SetHeaderOnly:
                      handleSetHeaderOnly(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::SetLensPosition:
                      handleSetLensPosition(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::SetExposureTime:
                      handleSetExposureTime(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::SetFrameDuration:
                      handleSetFrameDuration(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::GetFrameDurationLimits:
                      handleGetFrameDurationLimits(ws);
                      break;
                    case command_parser::CommandKind::GetLensPositionLimits:
                      handleGetLensPositionLimits(ws);
                      break;
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

void WebSocketServer::handleGetState(uWS::WebSocket<false, true, int>* ws) {
  const char* state_str;
  switch (system_state.load()) {
    case CameraState::CONFIGURED: state_str = "configured"; break;
    case CameraState::RUNNING:    state_str = "running";    break;
    default:                      state_str = "idle";       break;
  }
  json response;
  response["type"]  = Protocol::TYPE_STATE;
  response["state"] = state_str;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleConfigure(uWS::WebSocket<false, true, int>* ws,
                                      const json& params) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::Configure, system_state)) {
    sendError(ws, "Cameras must be idle to configure");
    return;
  }

  CameraConfig config = command_parser::buildCameraConfig(params);

  LOG_INFO("WebSocketServer", "Configuring cameras with resolution " +
                                  std::to_string(config.width) + "x" +
                                  std::to_string(config.height));

  if (camera_manager->configureAll(config)) {
    system_state = CameraState::CONFIGURED;
    sendStatus(ws, "Configured: " + std::to_string(config.width) + "x" +
                       std::to_string(config.height) + " YUV420");
  } else {
    sendError(ws, "Failed to configure cameras");
  }
}

void WebSocketServer::handleUnconfigure(
    uWS::WebSocket<false, true, int>* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::Unconfigure, system_state)) {
    sendError(ws, "Unconfigure only allowed when CONFIGURED");
    return;
  }

  LOG_INFO("WebSocketServer", "Unconfiguring cameras");
  if (!camera_manager->unconfigureAll()) {
    sendError(ws, "Failed to unconfigure cameras");
    return;
  }
  system_state = CameraState::IDLE;
  sendStatus(ws, "Unconfigured: returned to IDLE");
}

void WebSocketServer::handleSetSaveMode(uWS::WebSocket<false, true, int>* ws,
                                        const json& msg) {
  auto config = command_parser::buildSaveConfig(msg);
  if (!config) {
    std::string mode = msg.value("mode", std::string("(missing)"));
    sendError(ws, "Invalid save mode: " + mode);
    return;
  }

  frame_saver->configure(*config);
  std::string mode = msg.value("mode", std::string());
  LOG_INFO("WebSocketServer", "Save mode configured: " + mode);
  sendStatus(ws, "Save mode configured: " + mode);
}

void WebSocketServer::handleStartCameras(uWS::WebSocket<false, true, int>* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StartCameras, system_state)) {
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
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StartStream, system_state)) {
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
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StopCameras, system_state)) {
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

void WebSocketServer::handleSetLensPosition(
    uWS::WebSocket<false, true, int>* ws, const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::SetLensPosition, system_state)) {
    sendError(ws, "set_lens_position requires CONFIGURED or RUNNING state");
    return;
  }
  if (!msg.contains("lens_position") || !msg["lens_position"].is_number()) {
    sendError(ws, "set_lens_position requires numeric 'lens_position' field");
    return;
  }
  float lp = msg["lens_position"].get<float>();
  if (!std::isfinite(lp) || lp > 32.0f) {
    sendError(ws,
              "lens_position out of range (expected < 0 for AF, or [0, 32])");
    return;
  }
  camera_manager->setLensPosition(lp);
  if (lp < 0.0f) {
    sendStatus(ws, "Lens position: continuous autofocus");
  } else {
    sendStatus(ws, "Lens position: manual @ " + std::to_string(lp) +
                       " dioptres");
  }
}

void WebSocketServer::handleSetExposureTime(
    uWS::WebSocket<false, true, int>* ws, const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::SetExposureTime, system_state)) {
    sendError(ws, "set_exposure_time requires CONFIGURED or RUNNING state");
    return;
  }
  if (!msg.contains("exposure_time") || !msg["exposure_time"].is_number()) {
    sendError(ws, "set_exposure_time requires numeric 'exposure_time' field (microseconds)");
    return;
  }
  int32_t et = msg["exposure_time"].get<int32_t>();
  if (et == 0 || et > 1'000'000) {
    sendError(ws, "exposure_time out of range (< 0 for auto AE, or [1, 1000000] \xc2\xb5s)");
    return;
  }
  camera_manager->setExposureTime(et);
  if (et < 0) {
    sendStatus(ws, "Exposure time: auto AE");
  } else {
    sendStatus(ws, "Exposure time: manual @ " + std::to_string(et) + " \xc2\xb5s");
  }
}

void WebSocketServer::handleSetFrameDuration(
    uWS::WebSocket<false, true, int>* ws, const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::SetFrameDuration, system_state)) {
    sendError(ws, "set_frame_duration requires CONFIGURED or RUNNING state");
    return;
  }
  if (!msg.contains("frame_duration") || !msg["frame_duration"].is_number()) {
    sendError(ws, "set_frame_duration requires numeric 'frame_duration' field (microseconds)");
    return;
  }
  int64_t fd = msg["frame_duration"].get<int64_t>();
  if (fd > 1'000'000'000) {
    sendError(ws, "frame_duration out of range (\xe2\x89\xa4 0 to unset, or [1, 1000000000] \xc2\xb5s)");
    return;
  }
  camera_manager->setFrameDuration(fd);
  if (fd <= 0) {
    sendStatus(ws, "Frame duration: unset");
  } else {
    sendStatus(ws, "Frame duration: locked @ " + std::to_string(fd) + " \xc2\xb5s");
  }
}

void WebSocketServer::handleGetFrameDurationLimits(
    uWS::WebSocket<false, true, int>* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::GetFrameDurationLimits, system_state)) {
    sendError(ws, "get_frame_duration_limits requires CONFIGURED or RUNNING state");
    return;
  }
  auto [hw_min, hw_max] = camera_manager->getFrameDurationLimitsHw();
  int64_t current = camera_manager->getCurrentFrameDuration();
  size_t num_cameras = camera_manager->getCameraCount();

  json response;
  response["type"] = Protocol::TYPE_FRAME_DURATION_LIMITS;
  response["min"] = hw_min;
  response["max"] = hw_max;
  response["num_cameras"] = num_cameras;
  if (current > 0) {
    response["current"] = {{"min", current}, {"max", current}};
  } else {
    response["current"] = nullptr;
  }
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleGetLensPositionLimits(
    uWS::WebSocket<false, true, int>* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::GetLensPositionLimits, system_state)) {
    sendError(ws, "get_lens_position_limits requires CONFIGURED or RUNNING state");
    return;
  }
  LensPositionLimits limits = camera_manager->getLensPositionLimitsHw();
  size_t num_cameras = camera_manager->getCameraCount();

  // NaN fields (fixed-focus module / control unavailable) serialise to JSON
  // null via nlohmann's default dump() handling, which clients read as "no
  // hardware-reported limit".
  json response;
  response["type"] = Protocol::TYPE_LENS_POSITION_LIMITS;
  response["min"] = limits.min;
  response["max"] = limits.max;
  response["default"] = limits.def;
  response["num_cameras"] = num_cameras;
  ws->send(response.dump(), uWS::OpCode::TEXT);
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
  stream_manager->clearSink();

  // Disarm the sink so any lambdas already enqueued on the loop (from the
  // streaming thread, before clearSink joined it) skip ws_->send when they
  // run — uWS will free the underlying WebSocket once this close handler
  // returns. The shared_ptr keeps the sink itself alive so those lambdas can
  // still safely decrement the buffered counter.
  if (active_sink) {
    active_sink->invalidate();
  }
  active_sink.reset();

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
