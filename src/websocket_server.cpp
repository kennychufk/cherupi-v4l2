#include "websocket_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string_view>
#include <vector>

#include "capacity.hpp"
#include "command_parser.hpp"

using json = nlohmann::json;

WebSocketServer::WebSocketServer() {
  camera_manager = std::make_unique<CameraManager>();
  frame_processor = std::make_unique<FrameProcessor>();
  stream_manager =
      std::make_unique<StreamManager>(camera_manager.get(), frame_processor.get());

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
                      handleDiscover(ws,
                                     cmd.message.value("params", json::object()));
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
                    case command_parser::CommandKind::SetProcessMode:
                      handleSetProcessMode(ws, cmd.message);
                      break;
                    case command_parser::CommandKind::TriggerCapture:
                      handleTriggerCapture(ws, cmd.message);
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

  // Join the watchdog before anything it observes is torn down.
  stopCaptureWatchdog();

  // Clean up any active connection
  cleanupConnection();

  if (app) {
    app->close();
    delete app;
    app = nullptr;
  }
}

void WebSocketServer::handleDiscover(uWS::WebSocket<false, true, int>* ws,
                                     const json& params) {
  std::string sensor = command_parser::buildSensorFilter(params);
  size_t count = camera_manager->discoverCameras(sensor);

  json response;
  response["type"] = Protocol::TYPE_DISCOVERY;
  response["cameras"] = json::array();

  for (size_t i = 0; i < count; i++) {
    json cam_info;
    cam_info["id"] = i;
    cam_info["type"] = camera_manager->getCamera(i)->getModel();
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

void WebSocketServer::handleSetProcessMode(uWS::WebSocket<false, true, int>* ws,
                                           const json& msg) {
  auto config = command_parser::buildProcessConfig(msg);
  if (!config) {
    std::string mode = msg.value("mode", std::string("(missing)"));
    sendError(ws, "Invalid process mode: " + mode);
    return;
  }

  frame_processor->configure(*config);
  std::string mode = msg.value("mode", std::string());
  LOG_INFO("WebSocketServer", "Process mode configured: " + mode);
  sendStatus(ws, "Process mode configured: " + mode);
}

void WebSocketServer::handleTriggerCapture(uWS::WebSocket<false, true, int>* ws,
                                           const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::TriggerCapture, system_state)) {
    sendError(ws, "trigger_capture requires RUNNING state");
    return;
  }
  // The request is meaningful only in trigger mode; in any other mode it is
  // rejected rather than silently changing what that mode saves.
  if (frame_processor->getMode() != ProcessMode::TRIGGER) {
    sendError(ws, "trigger_capture requires the 'trigger' process mode");
    return;
  }

  // Which cameras to capture from: every running camera by default, or just
  // the one named by an optional "camera_id".
  std::vector<uint32_t> camera_ids;
  if (msg.contains("camera_id") && msg["camera_id"].is_number_integer()) {
    int64_t requested = msg["camera_id"].get<int64_t>();
    if (requested < 0) {
      sendError(ws, "camera_id must be non-negative");
      return;
    }
    uint32_t camera_id = static_cast<uint32_t>(requested);
    Camera* camera = camera_manager->getCamera(camera_id);
    if (!camera || camera->getState() != CameraState::RUNNING) {
      sendError(ws, "Camera " + std::to_string(camera_id) + " is not running");
      return;
    }
    camera_ids.push_back(camera_id);
  } else {
    for (Camera* camera : camera_manager->getAllCameras()) {
      if (camera->getState() == CameraState::RUNNING) {
        camera_ids.push_back(camera->getId());
      }
    }
    if (camera_ids.empty()) {
      sendError(ws, "No running cameras to trigger");
      return;
    }
  }

  int skip_frames = frame_processor->getTriggerSkipFrames();
  if (msg.contains("skip_frames") && msg["skip_frames"].is_number_integer()) {
    skip_frames = std::max(0, msg["skip_frames"].get<int>());
  }

  // The ack is produced on a capture thread, so it must not touch `ws`
  // directly. Capture the sink's shared_ptr here (on the loop thread) and let
  // it marshal the send back onto the loop — it also no-ops safely if the
  // client disconnects while the trigger is pending.
  std::shared_ptr<UwsFrameSink> sink = active_sink;
  auto trigger_id = frame_processor->armTrigger(
      camera_ids, skip_frames, [sink](TriggerResult result) {
        json response;
        response["type"] = Protocol::TYPE_TRIGGER_RESULT;
        response["trigger_id"] = result.trigger_id;
        response["cancelled"] = result.cancelled;
        response["captures"] = json::array();
        for (const auto& capture : result.captures) {
          json entry;
          entry["camera_id"] = capture.camera_id;
          entry["frame_id"] = capture.frame_id;
          entry["filename"] = capture.filename;
          response["captures"].push_back(std::move(entry));
        }
        if (sink) sink->sendText(response.dump());
      });

  if (!trigger_id) {
    // The only other way to fail here is a processor that never started in
    // trigger mode — i.e. set_process_mode was issued after start_cameras, so
    // the pipeline is still running whatever mode was active at start.
    sendError(ws, frame_processor->hasPendingTrigger()
                      ? "A trigger capture is already pending"
                      : "Frame processor is not running the trigger mode; send "
                        "set_process_mode 'trigger' before start_cameras");
    return;
  }

  LOG_INFO("WebSocketServer",
           "Trigger capture " + std::to_string(*trigger_id) + " armed for " +
               std::to_string(camera_ids.size()) + " camera(s)");
}

void WebSocketServer::handleStartCameras(uWS::WebSocket<false, true, int>* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StartCameras, system_state)) {
    sendError(ws, "Cameras must be configured before starting");
    return;
  }

  // Refuse a mode that arithmetically cannot keep up before any of it runs.
  // frame_processor->start() has to come first because the budget it resolves
  // is one of the inputs, so the check reads the started processor and we tear
  // it back down if the answer is no.
  frame_processor->start();
  if (const std::string reason = checkCaptureFeasibility(); !reason.empty()) {
    frame_processor->stop();
    sendError(ws, reason);
    return;
  }

  // Set up the frame callback whenever any processing is active (detection
  // and/or saving); the processor itself decides what to do per frame.
  if (frame_processor->getMode() != ProcessMode::NONE) {
    LOG_INFO("WebSocketServer", "Frame processing enabled, setting up callback");
    camera_manager->setFrameCallback(
        [this](const FrameData& frame) { frame_processor->saveFrame(frame); });
  } else {
    LOG_INFO("WebSocketServer", "Frame processing disabled, no callback set");
    camera_manager->setFrameCallback(nullptr);
  }

  // Start all cameras
  if (camera_manager->startAll()) {
    system_state = CameraState::RUNNING;
    startCaptureWatchdog();
    sendStatus(ws, "All cameras started successfully");
  } else {
    frame_processor->stop();
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

  // Before anything else, so it cannot mistake an orderly shutdown for a
  // stalled sensor and fire a spurious capture_timeout.
  stopCaptureWatchdog();

  // Stop streaming first
  stream_manager->stopAllStreaming();

  // Stop all cameras
  camera_manager->stopAll();

  // Flush buffered frames if in buffer mode
  if (frame_processor->getMode() == ProcessMode::BUFFER) {
    LOG_INFO("WebSocketServer", "Flushing buffered frames");
    frame_processor->flushBufferedFrames();
  }

  // Stop frame processor
  frame_processor->stop();

  system_state = CameraState::CONFIGURED;

  json status;
  status["type"] = Protocol::TYPE_STATUS;
  status["message"] = "All cameras stopped";
  status["frames_saved"] = frame_processor->getFramesSaved();
  status["bytes_written"] = frame_processor->getBytesWritten();
  // Non-zero means capture outran its sink and the recording has a gap. The
  // client needs this to know its dataset is incomplete.
  const size_t dropped = frame_processor->getFramesDroppedForBacklog();
  status["frames_dropped_backlog"] = dropped;
  if (dropped > 0) {
    LOG_WARN("WebSocketServer",
             "Dropped " + std::to_string(dropped) +
                 " frame(s) at the backlog budget of " +
                 capacity::formatBytes(
                     frame_processor->getBacklogBudgetBytes()) +
                 " during this session");
  }

  ws->send(status.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::handleResetFrameCounts(
    uWS::WebSocket<false, true, int>* ws) {
  LOG_INFO("WebSocketServer", "Resetting frame counts for all cameras");

  camera_manager->resetFrameCounts();

  // Also reset frame processor counts
  if (frame_processor) {
    frame_processor->resetFramesSavedCounts();
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

void WebSocketServer::startCaptureWatchdog() {
  stopCaptureWatchdog();
  watchdog_stop = false;
  capture_watchdog = std::thread(&WebSocketServer::captureWatchdogLoop, this);
  LOG_INFO("WebSocketServer", "Capture watchdog started");
}

void WebSocketServer::stopCaptureWatchdog() {
  watchdog_stop = true;
  if (capture_watchdog.joinable()) capture_watchdog.join();
}

void WebSocketServer::captureWatchdogLoop() {
  using namespace std::chrono_literals;
  while (!watchdog_stop.load()) {
    // 250 ms ticks: fast enough that the report follows the frontend timeout
    // closely, slow enough to be free. Camera::captureStalledForUs does the
    // actual threshold arithmetic per camera.
    std::this_thread::sleep_for(250ms);
    if (watchdog_stop.load()) break;
    for (const auto& stalled : camera_manager->pollStalledCameras()) {
      reportCaptureStall(stalled);
    }
  }
  LOG_DEBUG("WebSocketServer", "Capture watchdog stopped");
}

void WebSocketServer::reportCaptureStall(
    const CameraManager::StalledCamera& stalled) {
  const double seconds = static_cast<double>(stalled.stalled_for_us) / 1e6;
  std::ostringstream detail;
  detail << "Camera " << stalled.camera_id << " has delivered no frame for "
         << std::fixed << std::setprecision(1) << seconds
         << " s. The usual cause is libcamera's frontend watchdog firing "
            "(\"Camera frontend has timed out!\"), which is unrecoverable - "
            "capture never resumes on its own. Issue stop_cameras then "
            "start_cameras to rebuild the pipeline (reconfiguring is not "
            "needed). If frames resume by themselves this was a transient "
            "stall and no action is required.";
  LOG_ERROR("WebSocketServer", detail.str());

  // The most common cause on a small-memory host is the backlog having eaten
  // the machine, so say so when the numbers point that way — it turns a
  // mysterious stall into an actionable one.
  const size_t dropped = frame_processor->getFramesDroppedForBacklog();
  json response;
  response["type"] = Protocol::TYPE_ERROR;
  response["code"] = "capture_timeout";
  response["camera_id"] = stalled.camera_id;
  response["stalled_for_us"] = stalled.stalled_for_us;
  response["message"] = detail.str();
  response["backlog_bytes"] = frame_processor->getBacklogBytes();
  response["backlog_budget_bytes"] = frame_processor->getBacklogBudgetBytes();
  response["frames_dropped_backlog"] = dropped;

  // Any send from this thread must be marshalled onto the uWS loop; sendText
  // does that and no-ops safely if the client has already gone away.
  if (auto sink = active_sink) sink->sendText(response.dump());
}

std::string WebSocketServer::checkCaptureFeasibility() {
  const ProcessConfig& cfg = frame_processor->getConfig();
  const size_t camera_count = camera_manager->getCameraCount();
  if (cfg.mode == ProcessMode::NONE || camera_count == 0) return {};

  Camera* first = camera_manager->getCamera(0);
  if (!first) return {};
  const CameraConfig& geometry = first->getConfig();

  const auto [hw_min, hw_max] = camera_manager->getFrameDurationLimitsHw();
  const capacity::Demand demand = capacity::computeDemand(
      geometry.width, geometry.height, camera_count,
      camera_manager->getCurrentFrameDuration(), hw_min);

  LOG_INFO("WebSocketServer",
           "Capture demand: " + capacity::formatBytes(demand.bytes_per_frame) +
               "/frame x " + std::to_string(demand.frames_per_second).substr(0, 5) +
               " fps x " + std::to_string(camera_count) + " camera(s) = " +
               capacity::formatRate(demand.bytes_per_second));

  std::vector<capacity::Verdict> verdicts;
  verdicts.push_back(capacity::checkBufferHeadroom(
      cfg.mode, cfg.save_frames, demand,
      frame_processor->getBacklogBudgetBytes()));

  // Only probe the disk when a verdict actually depends on it — the probe
  // writes 32 MiB O_DIRECT, which costs over a second on an SD card.
  if (cfg.mode == ProcessMode::BATCH && cfg.save_frames) {
    const size_t disk_rate =
        cfg.disk_write_bytes_per_sec > 0
            ? cfg.disk_write_bytes_per_sec
            : capacity::diskWriteBytesPerSecond(
                  frame_processor->getActualOutputDir());
    verdicts.push_back(capacity::checkDiskThroughput(
        cfg.mode, cfg.save_frames, demand, disk_rate));
  }

  for (const auto& verdict : verdicts) {
    if (verdict.feasible) continue;
    if (cfg.allow_overcommit) {
      LOG_WARN("WebSocketServer",
               "Starting anyway (allow_overcommit): " + verdict.reason);
      continue;
    }
    return verdict.reason;
  }
  return {};
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

  // Note: We do NOT stop cameras here to allow frame processing to continue
  // Cameras will only be stopped by explicit stop_cameras command

  // Log current state
  if (system_state == CameraState::RUNNING) {
    LOG_INFO("WebSocketServer",
             "Cameras still running for potential frame processing");
  }

  // Reset client flag
  has_client = false;
}
