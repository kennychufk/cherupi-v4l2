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

  commander_.role = ClientRole::COMMANDER;
  commander_.stream = std::make_unique<StreamManager>(
      camera_manager.get(), frame_processor.get(), ClientRole::COMMANDER);
  observer_.role = ClientRole::OBSERVER;
  observer_.stream = std::make_unique<StreamManager>(
      camera_manager.get(), frame_processor.get(), ClientRole::OBSERVER);
  // Streaming priority: the observer's session only hands bytes to its socket
  // while the commander's connection is clear (see stream_priority.hpp).
  observer_.stream->setYieldTo(commander_.stream.get());

  // Stacking pins a camera's exposure/gain through the camera itself; the
  // cameras are discovered later, so look them up per call.
  raw_stacker = std::make_unique<RawStacker>(
      [this](uint32_t id, std::optional<int32_t> exposure,
             std::optional<float> gain) {
        if (Camera* camera = camera_manager->getCamera(id))
          camera->setCaptureOverride(exposure, gain);
      },
      [this](uint32_t id) {
        if (Camera* camera = camera_manager->getCamera(id))
          camera->clearCaptureOverride();
      });
  // A triggered frame gets the process mode's detection, like a live one.
  trigger_capturer = std::make_unique<TriggerCapturer>(
      [this](const FrameData& frame) {
        return frame_processor->detectCaptured(frame);
      });

  // Set up the callback for frame notifications: both sessions consume the
  // same publication slots, each with its own cursor.
  camera_manager->setStreamManagerNotify([this]() {
    if (commander_.stream) commander_.stream->notifyFrameAvailable();
    if (observer_.stream) observer_.stream->notifyFrameAvailable();
  });
}

WebSocketServer::~WebSocketServer() {
  LOG_INFO("WebSocketServer", "Shutting down server");
  stop();
  // The cameras' completion callbacks reach into the stacker and the frame
  // processor, which are destroyed before the camera manager: stop capture
  // first so none can fire into a destroyed object.
  if (camera_manager && camera_manager->areAnyRunning()) camera_manager->stopAll();
}

std::shared_ptr<UwsFrameSink> WebSocketServer::sinkOf(ClientSession& s) {
  std::lock_guard<std::mutex> lock(s.sink_mutex);
  return s.sink;
}

void WebSocketServer::run() {
  LOG_INFO("WebSocketServer",
           "Starting WebSocket server on port " + std::to_string(PORT));

  app = new uWS::App();

  app->ws<UwsSocketData>(
         "/*",
         {// Settings
          .compression = uWS::DISABLED,
          .maxPayloadLength = 16 * 1024 * 1024,  // 16MB max
          .idleTimeout = 120,
          .maxBackpressure = 1024 * 1024 * 10,  // 10MB

          // Handlers
          .upgrade =
              [this](auto* res, auto* req, auto* context) {
                // The path decides the role; each role has exactly one slot.
                const ClientRole role =
                    command_parser::roleForPath(req->getUrl());
                ClientSession& s = session(role);
                if (s.connected.load()) {
                  LOG_WARN("WebSocketServer",
                           std::string("Rejecting connection - server already "
                                       "has a ") +
                               clientRoleName(role) + " client");
                  res->writeStatus("503 Service Unavailable");
                  res->writeHeader("Content-Type", "text/plain");
                  res->end(std::string("Server already has a ") +
                           clientRoleName(role) + " client");
                  return;
                }

                // Accept the connection, tagging the socket with its role.
                UwsSocketData data;
                data.role = role;
                res->template upgrade<UwsSocketData>(
                    std::move(data), req->getHeader("sec-websocket-key"),
                    req->getHeader("sec-websocket-protocol"),
                    req->getHeader("sec-websocket-extensions"), context);
              },

          .open =
              [this](auto* ws) {
                const ClientRole role = ws->getUserData()->role;
                ClientSession& s = session(role);
                LOG_INFO("WebSocketServer",
                         std::string(clientRoleName(role)) +
                             " connected from " +
                             std::string(ws->getRemoteAddressAsText()));
                s.connected = true;
                auto sink = std::make_shared<UwsFrameSink>(ws);
                {
                  std::lock_guard<std::mutex> lock(s.sink_mutex);
                  s.sink = sink;
                }
                s.stream->setSink(sink.get());
                // Let the other client know its counterpart arrived.
                pushStateTo(otherRole(role), role == ClientRole::COMMANDER
                                                 ? "commander_connected"
                                                 : "observer_connected");
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
                const std::string cmd_name =
                    cmd.message.value("cmd", std::string());
                const ClientRole role = ws->getUserData()->role;
                LOG_DEBUG("WebSocketServer",
                          std::string("Received command from ") +
                              clientRoleName(role) + ": " + cmd_name);

                // Role gate first: an observer is read-only, and a refused
                // command must not even be evaluated against the state.
                if (!command_parser::isCommandPermitted(cmd.kind, role)) {
                  sendError(ws,
                            "Command '" + cmd_name +
                                "' is not permitted for an observer client "
                                "(read-only); only the commander may issue it",
                            Protocol::ERR_CODE_FORBIDDEN);
                  return;
                }

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
                    case command_parser::CommandKind::StackCapture:
                      handleStackCapture(ws, cmd.message);
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
                    case command_parser::CommandKind::GetFocusFomRegion:
                      handleGetFocusFomRegion(ws);
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
                // Handle backpressure with hysteresis to prevent oscillation.
                // The state lives in the socket's user data: the commander
                // and the observer are separate connections with separate
                // congestion, so each gets its own.
                UwsSocketData* data = ws->getUserData();
                ClientSession& s = session(data->role);
                const size_t bufferedAmount = ws->getBufferedAmount();

                // Different thresholds for entering and leaving backpressure
                static constexpr size_t BACKPRESSURE_HIGH_WATER =
                    512 * 1024;  // 512KB
                static constexpr size_t BACKPRESSURE_LOW_WATER =
                    128 * 1024;  // 128KB

                const bool hasPressure =
                    data->in_backpressure
                        ? bufferedAmount > BACKPRESSURE_LOW_WATER
                        : bufferedAmount > BACKPRESSURE_HIGH_WATER;

                // Only log and notify on state changes
                if (hasPressure != data->in_backpressure) {
                  LOG_DEBUG("WebSocketServer",
                            std::string(clientRoleName(data->role)) +
                                (hasPressure ? ": entering" : ": exiting") +
                                " backpressure state: " +
                                std::to_string(bufferedAmount) +
                                " bytes buffered (threshold: " +
                                std::to_string(hasPressure
                                                   ? BACKPRESSURE_HIGH_WATER
                                                   : BACKPRESSURE_LOW_WATER) +
                                ")");
                  data->in_backpressure = hasPressure;
                  s.stream->notifyBackpressure(hasPressure);
                }

                // Periodic status if the buffer is very high
                if (bufferedAmount > 1024 * 1024) {  // 1MB
                  auto now = std::chrono::steady_clock::now();
                  if (now - s.last_high_buffer_log > std::chrono::seconds(1)) {
                    LOG_WARN("WebSocketServer",
                             std::string(clientRoleName(data->role)) +
                                 ": high buffer warning: " +
                                 std::to_string(bufferedAmount / 1024) +
                                 " KB buffered");
                    s.last_high_buffer_log = now;
                  }
                }
              },

          .close =
              [this](auto* ws, int code, std::string_view message) {
                const ClientRole role = ws->getUserData()->role;
                LOG_INFO("WebSocketServer",
                         std::string(clientRoleName(role)) +
                             " disconnected with code " + std::to_string(code) +
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

                cleanupConnection(session(role));
                pushStateTo(otherRole(role), role == ClientRole::COMMANDER
                                                 ? "commander_disconnected"
                                                 : "observer_disconnected");
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

  // Clean up any active connections
  cleanupConnection(commander_);
  cleanupConnection(observer_);

  if (app) {
    app->close();
    delete app;
    app = nullptr;
  }
}

void WebSocketServer::handleDiscover(Socket* ws, const json& params) {
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

const char* WebSocketServer::stateName(CameraState state) {
  switch (state) {
    case CameraState::CONFIGURED: return "configured";
    case CameraState::RUNNING:    return "running";
    default:                      return "idle";
  }
}

json WebSocketServer::stateMessage(ClientRole recipient, const char* cause) {
  json m;
  m["type"] = Protocol::TYPE_STATE;
  m["state"] = stateName(system_state.load());
  m["cause"] = cause;
  m["role"] = clientRoleName(recipient);
  m["commander_connected"] = commander_.connected.load();
  m["observer_connected"] = observer_.connected.load();
  return m;
}

void WebSocketServer::pushStateTo(ClientRole role, const char* cause) {
  // Goes through the sink so it is safe from any thread and no-ops if that
  // client is not connected (or is going away).
  if (auto sink = sinkOf(session(role))) {
    sink->sendText(stateMessage(role, cause).dump());
  }
}

void WebSocketServer::broadcastState(const char* cause) {
  pushStateTo(ClientRole::COMMANDER, cause);
  pushStateTo(ClientRole::OBSERVER, cause);
}

void WebSocketServer::handleGetState(Socket* ws) {
  ws->send(stateMessage(ws->getUserData()->role, "query").dump(),
           uWS::OpCode::TEXT);
}

void WebSocketServer::handleConfigure(Socket* ws, const json& params) {
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
    // The FocusFoM region rides on the configure reply because it is fixed
    // by the configured sensor mode; observers (and later reconnects) read
    // it with get_focus_fom_region.
    json response;
    response["type"] = Protocol::TYPE_STATUS;
    response["message"] = "Configured: " + std::to_string(config.width) + "x" +
                          std::to_string(config.height) + " YUV420";
    response["focus_fom_region"] = focusFomRegionJson();
    // The raw stream stack_capture reads, when asked for; null otherwise.
    response["raw"] = nullptr;
    if (Camera* camera = camera_manager->getCamera(0)) {
      const Camera::RawStreamInfo& raw = camera->getRawStreamInfo();
      if (raw.enabled) {
        response["raw"] = {{"format", raw.format},
                           {"width", raw.width},
                           {"height", raw.height},
                           {"stride", raw.stride},
                           {"bit_depth", raw.bit_depth}};
      }
    }
    ws->send(response.dump(), uWS::OpCode::TEXT);
    broadcastState("configure");
  } else {
    sendError(ws, "Failed to configure cameras");
  }
}

void WebSocketServer::handleUnconfigure(Socket* ws) {
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
  broadcastState("unconfigure");
}

void WebSocketServer::handleSetProcessMode(Socket* ws, const json& msg) {
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

bool WebSocketServer::captureInProgress() {
  return capture_busy_.load() || commander_.stream->hasPendingCaptures();
}

void WebSocketServer::deliverCapture(std::vector<CaptureFrame> frames) {
  // Queue first, then release: the queue holds the gate from here on, so
  // there is no moment in which a second capture could slip in.
  commander_.stream->enqueueCaptures(std::move(frames));
  capture_busy_ = false;
}

void WebSocketServer::updateLiveFrames() {
  const bool needed = frame_processor->getMode() != ProcessMode::NONE ||
                      commander_.stream->getStreamingCameraCount() > 0 ||
                      observer_.stream->getStreamingCameraCount() > 0 ||
                      trigger_capturer->hasPending();
  camera_manager->setLiveFrames(needed);
}

void WebSocketServer::handleTriggerCapture(Socket* ws, const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::TriggerCapture, system_state)) {
    sendError(ws, "trigger_capture requires RUNNING state");
    return;
  }
  auto parsed = command_parser::buildTriggerRequest(msg);
  if (std::holds_alternative<std::string>(parsed)) {
    sendError(ws, std::get<std::string>(parsed));
    return;
  }
  const TriggerRequest& req = std::get<TriggerRequest>(parsed);

  // Which cameras to capture from: every running camera by default, or just
  // the one named by an optional "camera_id".
  TriggerPlan plan;
  if (req.camera_id) {
    Camera* camera = camera_manager->getCamera(*req.camera_id);
    if (!camera || camera->getState() != CameraState::RUNNING) {
      sendError(ws, "Camera " + std::to_string(*req.camera_id) + " is not running");
      return;
    }
    plan.camera_ids.push_back(*req.camera_id);
  } else {
    for (Camera* camera : camera_manager->getAllCameras())
      if (camera->getState() == CameraState::RUNNING)
        plan.camera_ids.push_back(camera->getId());
    if (plan.camera_ids.empty()) {
      sendError(ws, "No running cameras to trigger");
      return;
    }
  }
  plan.skip_frames = req.skip_frames;
  plan.send_image = req.send_image;
  plan.detection_kind = detectionKindForMode(frame_processor->getMode());

  if (captureInProgress()) {
    sendError(ws, "A capture is already in progress: wait for its frames "
                  "before the next trigger_capture or stack_capture");
    return;
  }
  capture_busy_ = true;
  // The triggered frame is a copy of a live frame, so the cameras must be
  // copying them out before the trigger can see one.
  camera_manager->setLiveFrames(true);

  // The ack comes from a capture thread and the frames from the capturer's
  // worker, so neither may touch `ws`: the ack goes through the sink (which
  // marshals onto the loop and no-ops once the client has gone) and the
  // frames through the commander's stream. Only a commander gets here (role
  // gate).
  std::shared_ptr<UwsFrameSink> sink = sinkOf(sessionFor(ws));
  const size_t num_cameras = plan.camera_ids.size();
  auto trigger_id = trigger_capturer->arm(
      std::move(plan),
      [this, sink](TriggerResult result) {
        if (sink) sink->sendText(triggerResultJson(result).dump());
        updateLiveFrames();
      },
      [this](std::vector<CaptureFrame> frames) { deliverCapture(std::move(frames)); });
  if (!trigger_id) {
    capture_busy_ = false;
    updateLiveFrames();
    sendError(ws, "A trigger capture is already pending");
    return;
  }
  LOG_INFO("WebSocketServer",
           "Trigger capture " + std::to_string(*trigger_id) + " armed for " +
               std::to_string(num_cameras) + " camera(s)");
}

bool WebSocketServer::rawConfigured() {
  Camera* camera = camera_manager->getCamera(0);
  return camera && camera->getRawStreamInfo().enabled;
}

void WebSocketServer::handleStackCapture(Socket* ws, const json& msg) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StackCapture, system_state)) {
    sendError(ws, "stack_capture requires RUNNING state");
    return;
  }
  if (!rawConfigured()) {
    sendError(ws, "stack_capture needs the raw stream: configure with "
                  "\"raw\": true");
    return;
  }
  auto parsed = command_parser::buildStackRequest(msg);
  if (std::holds_alternative<std::string>(parsed)) {
    sendError(ws, std::get<std::string>(parsed));
    return;
  }
  const StackRequest& req = std::get<StackRequest>(parsed);

  StackPlan plan;
  if (req.camera_id) {
    Camera* camera = camera_manager->getCamera(*req.camera_id);
    if (!camera || camera->getState() != CameraState::RUNNING) {
      sendError(ws, "Camera " + std::to_string(*req.camera_id) + " is not running");
      return;
    }
    plan.camera_ids.push_back(*req.camera_id);
  } else {
    for (Camera* camera : camera_manager->getAllCameras())
      if (camera->getState() == CameraState::RUNNING)
        plan.camera_ids.push_back(camera->getId());
    if (plan.camera_ids.empty()) {
      sendError(ws, "No running cameras to stack");
      return;
    }
  }

  // An exposure longer than a locked frame duration would be clamped and
  // never settle; say so now rather than after 30 frames.
  if (req.exposure_time_us) {
    const int64_t locked = camera_manager->getCurrentFrameDuration();
    if (locked > 0 && *req.exposure_time_us > locked) {
      sendError(ws, "exposure_time " + std::to_string(*req.exposure_time_us) +
                        " us exceeds the locked frame duration of " +
                        std::to_string(locked) +
                        " us; unlock it with set_frame_duration 0 first");
      return;
    }
  }

  // The process mode's detection, with its parameters.
  const ProcessConfig& pc = frame_processor->getConfig();
  plan.num_frames = req.num_frames;
  plan.exposure_time_us = req.exposure_time_us;
  plan.analogue_gain = req.analogue_gain;
  plan.skip_frames = req.skip_frames;
  plan.send_image = req.send_image;
  plan.detect.kind = isDetectorMode(pc.mode) ? pc.mode : ProcessMode::NONE;
  plan.detect.rows = pc.checkerboard_rows;
  plan.detect.cols = pc.checkerboard_cols;
  // The window is set in full-res pixels; the stack is 2x2-binned.
  plan.detect.subpix_half_window = pc.checkerboard_subpix_window / 2;
  const bool aruco = plan.detect.kind == ProcessMode::ARUCO ||
                     plan.detect.kind == ProcessMode::ARUCO2X2;
  plan.detect.num_threads = aruco ? pc.aruco_num_threads : pc.checkerboard_num_threads;
  plan.detect.aruco_corner_refine = pc.aruco_corner_refine;
  plan.detect.aruco_edge_refine = pc.aruco_edge_refine;

  if (captureInProgress()) {
    sendError(ws, "A capture is already in progress: wait for its frames "
                  "before the next trigger_capture or stack_capture");
    return;
  }
  capture_busy_ = true;

  // The ack and the images come from the stacker's worker thread; like the
  // trigger's they go out through the commander's sink and stream.
  std::shared_ptr<UwsFrameSink> sink = sinkOf(sessionFor(ws));
  const size_t num_cameras = plan.camera_ids.size();
  auto stack_id = raw_stacker->arm(
      std::move(plan),
      [sink](StackResult result) {
        if (sink) sink->sendText(stackResultJson(result).dump());
      },
      [this](std::vector<CaptureFrame> frames) { deliverCapture(std::move(frames)); });
  if (!stack_id) {
    capture_busy_ = false;
    sendError(ws, "A stack capture is already pending");
    return;
  }
  LOG_INFO("WebSocketServer",
           "Stack capture " + std::to_string(*stack_id) + " armed for " +
               std::to_string(num_cameras) + " camera(s), " +
               std::to_string(req.num_frames) + " frames");
}

void WebSocketServer::handleStartCameras(Socket* ws) {
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

  // Every live frame goes past the trigger capturer (which ignores it unless
  // a trigger is armed) and the processor, which decides per mode what to do
  // with it — nothing in mode none.
  camera_manager->setFrameCallback(
      [this](const std::shared_ptr<const FrameData>& frame) {
        trigger_capturer->onFrame(*frame);
        frame_processor->saveFrame(frame);
      });

  // Raw frames feed the stacker, which ignores them until a stack is armed.
  if (rawConfigured()) {
    camera_manager->setRawFrameCallback(
        [this](const RawFrame& frame) { raw_stacker->onRawFrame(frame); });
  } else {
    camera_manager->setRawFrameCallback(nullptr);
  }
  raw_stacker->reset();
  trigger_capturer->reset();
  updateLiveFrames();

  // Start all cameras
  if (camera_manager->startAll()) {
    system_state = CameraState::RUNNING;
    startCaptureWatchdog();
    sendStatus(ws, "All cameras started successfully");
    broadcastState("start_cameras");
  } else {
    frame_processor->stop();
    sendError(ws, "Failed to start cameras");
  }
}

void WebSocketServer::handleStartStream(Socket* ws, const json& msg) {
  const ClientRole role = ws->getUserData()->role;
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StartStream, system_state, role)) {
    sendError(ws, "Cameras must be running to start streaming");
    return;
  }

  auto parsed = command_parser::buildStreamRequest(msg);
  if (std::holds_alternative<std::string>(parsed)) {
    sendError(ws, std::get<std::string>(parsed));
    return;
  }
  const auto& request = std::get<command_parser::StreamRequest>(parsed);

  const size_t camera_count = camera_manager->getCameraCount();
  if (request.camera_id >= camera_count) {
    sendError(ws, "Unknown camera_id " + std::to_string(request.camera_id) +
                      (camera_count == 0
                           ? " (no cameras discovered yet; send discover first)"
                           : " (discovered cameras: 0.." +
                                 std::to_string(camera_count - 1) + ")"));
    return;
  }

  ClientSession& s = sessionFor(ws);
  if (s.stream->startStreamingCamera(request.camera_id, request.options)) {
    updateLiveFrames();
    std::ostringstream text;
    text << "Started streaming camera " << request.camera_id;
    if (request.options.subsample > 1) {
      text << " (subsample " << request.options.subsample << ")";
    }
    if (request.options.max_fps > 0.0) {
      text << " (max " << request.options.max_fps << " fps)";
    }
    if (role == ClientRole::OBSERVER &&
        system_state.load() != CameraState::RUNNING) {
      text << " - subscribed; frames will flow once the cameras are running";
    }
    LOG_INFO("WebSocketServer", std::string(clientRoleName(role)) + ": " +
                                    text.str());
    sendStatus(ws, text.str());
  } else {
    sendError(ws, "Failed to start streaming camera " +
                      std::to_string(request.camera_id));
  }
}

void WebSocketServer::handleStopStream(Socket* ws, const json& msg) {
  auto parsed = command_parser::buildStreamRequest(msg);
  if (std::holds_alternative<std::string>(parsed)) {
    sendError(ws, std::get<std::string>(parsed));
    return;
  }
  const uint32_t camera_id =
      std::get<command_parser::StreamRequest>(parsed).camera_id;

  ClientSession& s = sessionFor(ws);
  if (s.stream->stopStreamingCamera(camera_id)) {
    updateLiveFrames();
    LOG_INFO("WebSocketServer",
             std::string(clientRoleName(s.role)) +
                 ": stopped streaming camera " + std::to_string(camera_id));
    sendStatus(ws, "Stopped streaming camera " + std::to_string(camera_id));
  } else {
    sendError(ws, "Camera " + std::to_string(camera_id) + " was not streaming");
  }
}

void WebSocketServer::handleStopCameras(Socket* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::StopCameras, system_state)) {
    sendError(ws, "Cameras are not running");
    return;
  }

  LOG_INFO("WebSocketServer", "Stopping all cameras");

  // Before anything else, so it cannot mistake an orderly shutdown for a
  // stalled sensor and fire a spurious capture_timeout.
  stopCaptureWatchdog();

  // Drop the commander's subscriptions first. Its streaming thread keeps
  // running so the frames of a capture already taken still go out. The
  // observer's subscriptions are deliberately left alone: they are
  // persistent, and its streamer simply finds no new frames until the next
  // start_cameras.
  commander_.stream->clearSubscriptions();

  // A capture still waiting on cameras can never finish now: a trigger
  // delivers the frames it already has, a stack hands its cameras back their
  // preview settings, and each tells the client it was cancelled.
  trigger_capturer->cancel();
  raw_stacker->cancel();

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
  broadcastState("stop_cameras");
}

void WebSocketServer::handleResetFrameCounts(Socket* ws) {
  LOG_INFO("WebSocketServer", "Resetting frame counts for all cameras");

  camera_manager->resetFrameCounts();

  // Also reset frame processor counts
  if (frame_processor) {
    frame_processor->resetFramesSavedCounts();
  }

  sendStatus(ws, "Frame counts reset for all cameras");
}

void WebSocketServer::handleSetHeaderOnly(Socket* ws, const json& msg) {
  bool enabled = msg.value("enabled", false);

  ClientSession& s = sessionFor(ws);
  LOG_INFO("WebSocketServer",
           std::string(clientRoleName(s.role)) +
               ": setting header only mode to " +
               std::string(enabled ? "enabled" : "disabled"));

  // Per connection: each role's session has its own flag.
  s.stream->setHeaderOnlyMode(enabled);

  // Send confirmation status
  sendStatus(
      ws, "Header only mode " + std::string(enabled ? "enabled" : "disabled"));
}

void WebSocketServer::handleSetLensPosition(Socket* ws, const json& msg) {
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

void WebSocketServer::handleSetExposureTime(Socket* ws, const json& msg) {
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

void WebSocketServer::handleSetFrameDuration(Socket* ws, const json& msg) {
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

void WebSocketServer::handleGetFrameDurationLimits(Socket* ws) {
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

void WebSocketServer::handleGetLensPositionLimits(Socket* ws) {
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

json WebSocketServer::focusFomRegionJson() const {
  const auto region = camera_manager->getFocusFomRegion();
  if (!region) return nullptr;
  json j;
  j["x"] = region->frame.x;
  j["y"] = region->frame.y;
  j["width"] = region->frame.width;
  j["height"] = region->frame.height;
  j["frame_width"] = region->frame_width;
  j["frame_height"] = region->frame_height;
  j["grid"] = {{"size", region->grid_size},
               {"first_col", region->first_col},
               {"first_row", region->first_row},
               {"num_cols", region->num_cols},
               {"num_rows", region->num_rows}};
  return j;
}

void WebSocketServer::handleGetFocusFomRegion(Socket* ws) {
  if (!command_parser::isCommandAllowed(
          command_parser::CommandKind::GetFocusFomRegion, system_state)) {
    sendError(ws, "get_focus_fom_region requires CONFIGURED or RUNNING state");
    return;
  }
  json body = focusFomRegionJson();
  if (body.is_null()) {
    sendError(ws, "No cameras configured");
    return;
  }
  json response;
  response["type"] = Protocol::TYPE_FOCUS_FOM_REGION;
  response.update(body);
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
  response["code"] = Protocol::ERR_CODE_CAPTURE_TIMEOUT;
  response["camera_id"] = stalled.camera_id;
  response["stalled_for_us"] = stalled.stalled_for_us;
  response["message"] = detail.str();
  response["backlog_bytes"] = frame_processor->getBacklogBytes();
  response["backlog_budget_bytes"] = frame_processor->getBacklogBudgetBytes();
  response["frames_dropped_backlog"] = dropped;

  // Any send from this thread must be marshalled onto the uWS loop; sendText
  // does that and no-ops safely if a client has already gone away. Both roles
  // get it: the commander must recover, the observer should show the outage.
  const std::string payload = response.dump();
  if (auto sink = sinkOf(commander_)) sink->sendText(payload);
  if (auto sink = sinkOf(observer_)) sink->sendText(payload);
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

void WebSocketServer::sendStatus(Socket* ws, const std::string& message) {
  json response;
  response["type"] = Protocol::TYPE_STATUS;
  response["message"] = message;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::sendError(Socket* ws, const std::string& message,
                                const char* code) {
  LOG_ERROR("WebSocketServer", "Sending error: " + message);
  json response;
  response["type"] = Protocol::TYPE_ERROR;
  response["message"] = message;
  if (code) response["code"] = code;
  ws->send(response.dump(), uWS::OpCode::TEXT);
}

void WebSocketServer::cleanupConnection(ClientSession& s) {
  LOG_INFO("WebSocketServer",
           std::string("Cleaning up ") + clientRoleName(s.role) + " connection");

  // Stop this session's streaming (dropping its subscriptions) but keep the
  // cameras running: the other client may still be watching, and processing
  // must continue regardless.
  s.stream->clearSink();

  // Disarm the sink so any lambdas already enqueued on the loop (from the
  // streaming thread, before clearSink joined it) skip ws_->send when they
  // run — uWS will free the underlying WebSocket once this close handler
  // returns. The shared_ptr keeps the sink itself alive so those lambdas can
  // still safely decrement the buffered counter.
  {
    std::lock_guard<std::mutex> lock(s.sink_mutex);
    if (s.sink) s.sink->invalidate();
    s.sink.reset();
  }

  // Note: We do NOT stop cameras here to allow frame processing to continue
  // Cameras will only be stopped by explicit stop_cameras command
  if (system_state == CameraState::RUNNING) {
    LOG_INFO("WebSocketServer",
             "Cameras still running for potential frame processing");
  }

  // Release the slot
  s.connected = false;
  // Its subscriptions went with it.
  updateLiveFrames();
}
