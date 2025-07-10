#include "websocket_server.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string_view>

using json = nlohmann::json;

WebSocketServer::WebSocketServer() {
  camera_manager = std::make_unique<CameraManager>();
  frame_saver = std::make_unique<FrameSaver>();
  stream_manager = std::make_unique<StreamManager>(camera_manager.get());
}

WebSocketServer::~WebSocketServer() { stop(); }

void WebSocketServer::run() {
  std::cout << "Starting WebSocket server on port " << PORT << "..."
            << std::endl;

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
                std::cout << "Client connected" << std::endl;
                has_client = true;
                stream_manager->setWebSocket(ws);

                // Don't set the frame callback here - it should be set
                // after configuration and before starting cameras
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
                  } else {
                    sendError(ws, "Unknown command: " + cmd);
                  }
                } catch (json::exception& e) {
                  sendError(ws, std::string("JSON parse error: ") + e.what());
                }
              },

          .drain =
              [this](auto* ws) {
                // Handle backpressure
                stream_manager->notifyBackpressure(ws->getBufferedAmount() > 0);
              },

          .close =
              [this](auto* ws, int code, std::string_view message) {
                std::cout << "Client disconnected with code " << code
                          << std::endl;
                cleanupConnection();
              }})
      .listen(PORT, [this](auto* listen_socket) {
        if (listen_socket) {
          std::cout << "WebSocket server listening on port " << PORT
                    << std::endl;
        } else {
          std::cerr << "Failed to listen on port " << PORT << std::endl;
        }
      });

  // Run the event loop
  loop = uWS::Loop::get();
  app->run();
}

void WebSocketServer::stop() {
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
    cam_info["type"] = "IMX296";
    response["cameras"].push_back(cam_info);
  }

  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleConfigure(uWS::WebSocket<false, true, int>* ws,
                                      const json& params) {
  if (system_state != CameraState::IDLE) {
    sendError(ws, "Cameras must be idle to configure");
    return;
  }

  // For now, use default configuration
  CameraConfig config;

  // Override with any provided parameters
  if (params.contains("width")) config.width = params["width"];
  if (params.contains("height")) config.height = params["height"];
  if (params.contains("crop_width")) config.crop_width = params["crop_width"];
  if (params.contains("crop_height"))
    config.crop_height = params["crop_height"];
  if (params.contains("crop_left")) config.crop_left = params["crop_left"];
  if (params.contains("crop_top")) config.crop_top = params["crop_top"];

  if (camera_manager->configureAll(config)) {
    system_state = CameraState::CONFIGURED;
    sendStatus(ws, "All cameras configured successfully");
  } else {
    sendError(ws, "Failed to configure cameras");
  }
}

void WebSocketServer::handleSetSaveMode(uWS::WebSocket<false, true, int>* ws,
                                        const json& msg) {
  if (system_state == CameraState::RUNNING) {
    sendError(ws, "Cannot change save mode while cameras are running");
    return;
  }

  SaveConfig config;
  std::string mode = msg["mode"];

  if (mode == "none") {
    config.mode = SaveMode::NONE;
  } else if (mode == "buffer") {
    config.mode = SaveMode::BUFFER;
  } else if (mode == "batch") {
    config.mode = SaveMode::BATCH;
  } else {
    sendError(ws, "Invalid save mode: " + mode);
    return;
  }

  // Get optional parameters
  if (msg.contains("params")) {
    const json& params = msg["params"];
    if (params.contains("prefix")) config.prefix = params["prefix"];
    if (params.contains("batch_size")) config.batch_size = params["batch_size"];
    if (params.contains("writer_threads"))
      config.writer_threads = params["writer_threads"];
  }

  frame_saver->configure(config);
  sendStatus(ws, "Save mode configured: " + mode);
}

void WebSocketServer::handleStartCameras(uWS::WebSocket<false, true, int>* ws) {
  if (system_state != CameraState::CONFIGURED) {
    sendError(ws, "Cameras must be configured before starting");
    return;
  }

  // Start frame saver first
  frame_saver->start();

  // Set up frame callback for saving BEFORE starting cameras
  // This ensures the callback is properly connected
  camera_manager->setFrameCallback(
      [this](const FrameData& frame) { frame_saver->saveFrame(frame); });

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

  // Stop streaming first
  stream_manager->stopAllStreaming();

  // Stop all cameras
  camera_manager->stopAll();

  // Flush buffered frames if in buffer mode
  if (frame_saver->getMode() == SaveMode::BUFFER) {
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

void WebSocketServer::sendStatus(uWS::WebSocket<false, true, int>* ws,
                                 const std::string& message) {
  json response;
  response["type"] = Protocol::TYPE_STATUS;
  response["message"] = message;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::sendError(uWS::WebSocket<false, true, int>* ws,
                                const std::string& message) {
  json response;
  response["type"] = Protocol::TYPE_ERROR;
  response["message"] = message;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::cleanupConnection() {
  // Stop streaming
  stream_manager->clearWebSocket();

  // Stop cameras if running
  if (system_state == CameraState::RUNNING) {
    camera_manager->stopAll();

    // Flush buffered frames if needed
    if (frame_saver->getMode() == SaveMode::BUFFER) {
      frame_saver->flushBufferedFrames();
    }

    frame_saver->stop();
  }

  // Reset state
  system_state = CameraState::IDLE;
  has_client = false;
}
