#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "camera_manager.hpp"
#include "frame_processor.hpp"
#include "stream_manager.hpp"
#include "types.hpp"
#include "uws_frame_sink.hpp"

// Include uWebSockets headers directly
#include <uWS/App.h>

// Include nlohmann/json
#include <nlohmann/json.hpp>

class WebSocketServer {
 private:
  static constexpr int PORT = 9001;

  std::unique_ptr<CameraManager> camera_manager;
  std::unique_ptr<FrameProcessor> frame_processor;
  std::unique_ptr<StreamManager> stream_manager;
  std::shared_ptr<UwsFrameSink> active_sink;

  std::atomic<bool> has_client{false};
  std::atomic<CameraState> system_state{CameraState::IDLE};

  // uWebSockets app
  uWS::App* app = nullptr;
  uWS::Loop* loop = nullptr;

  // Message handlers
  void handleDiscover(uWS::WebSocket<false, true, int>* ws,
                      const nlohmann::json& params);
  void handleGetState(uWS::WebSocket<false, true, int>* ws);
  void handleConfigure(uWS::WebSocket<false, true, int>* ws,
                       const nlohmann::json& params);
  void handleUnconfigure(uWS::WebSocket<false, true, int>* ws);
  void handleSetProcessMode(uWS::WebSocket<false, true, int>* ws,
                         const nlohmann::json& params);
  void handleTriggerCapture(uWS::WebSocket<false, true, int>* ws,
                            const nlohmann::json& msg);
  void handleStartCameras(uWS::WebSocket<false, true, int>* ws);
  void handleStartStream(uWS::WebSocket<false, true, int>* ws,
                         const nlohmann::json& params);
  void handleStopStream(uWS::WebSocket<false, true, int>* ws,
                        const nlohmann::json& params);
  void handleStopCameras(uWS::WebSocket<false, true, int>* ws);
  void handleResetFrameCounts(uWS::WebSocket<false, true, int>* ws);
  void handleSetHeaderOnly(uWS::WebSocket<false, true, int>* ws,
                           const nlohmann::json& msg);
  void handleSetLensPosition(uWS::WebSocket<false, true, int>* ws,
                             const nlohmann::json& msg);
  void handleSetExposureTime(uWS::WebSocket<false, true, int>* ws,
                             const nlohmann::json& msg);
  void handleSetFrameDuration(uWS::WebSocket<false, true, int>* ws,
                              const nlohmann::json& msg);
  void handleGetFrameDurationLimits(uWS::WebSocket<false, true, int>* ws);
  void handleGetLensPositionLimits(uWS::WebSocket<false, true, int>* ws);

  void sendStatus(uWS::WebSocket<false, true, int>* ws,
                  const std::string& message);
  void sendError(uWS::WebSocket<false, true, int>* ws,
                 const std::string& message);

  void cleanupConnection();

  // --- Capture watchdog ---------------------------------------------------
  // libcamera's RPi pipeline handler abandons a stalled frontend ("Camera
  // frontend has timed out!") and then never calls requestCompleted again.
  // Nothing surfaces that to the application, so without this the server sits
  // there with an open socket and a dead sensor while the client waits
  // forever. The watchdog turns that silence into an `error` message.
  void startCaptureWatchdog();
  void stopCaptureWatchdog();
  void captureWatchdogLoop();
  // Report one stalled camera to the connected client, if any. Runs on the
  // watchdog thread, so it goes out through UwsFrameSink::sendText (which
  // marshals onto the uWS loop) rather than touching the socket directly.
  void reportCaptureStall(const CameraManager::StalledCamera& stalled);

  // Refuse to start when the configured process mode provably cannot keep up
  // with the camera on this hardware. Returns an empty string when start may
  // proceed, otherwise the operator-facing reason to send back. Bypassed by
  // ProcessConfig::allow_overcommit, which downgrades it to a warning.
  std::string checkCaptureFeasibility();

  std::thread capture_watchdog;
  std::atomic<bool> watchdog_stop{true};

 public:
  WebSocketServer();
  ~WebSocketServer();

  void run();
  void stop();
};
