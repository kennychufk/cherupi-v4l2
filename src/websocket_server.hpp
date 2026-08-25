#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
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
  using Socket = UwsFrameSink::Socket;

  // Everything kept per client role. The streaming session is created once
  // and outlives connections — a connection merely attaches its sink to it —
  // so subscriptions and options are scoped to a connection while the
  // machinery (thread, rate control, the observer→commander priority link) is
  // permanent. One session per role: the commander (read-write) and the
  // observer (read-only).
  struct ClientSession {
    ClientRole role = ClientRole::COMMANDER;
    std::unique_ptr<StreamManager> stream;
    std::atomic<bool> connected{false};
    // Guards `sink`: read on the loop thread (handlers) and on the watchdog
    // thread (reportCaptureStall); written on open/close.
    std::mutex sink_mutex;
    std::shared_ptr<UwsFrameSink> sink;
    // Throttle for the drain handler's high-buffer warning.
    std::chrono::steady_clock::time_point last_high_buffer_log{};
  };

  std::unique_ptr<CameraManager> camera_manager;
  std::unique_ptr<FrameProcessor> frame_processor;
  ClientSession commander_;
  ClientSession observer_;

  ClientSession& session(ClientRole role) {
    return role == ClientRole::OBSERVER ? observer_ : commander_;
  }
  ClientSession& sessionFor(Socket* ws) {
    return session(ws->getUserData()->role);
  }
  static ClientRole otherRole(ClientRole role) {
    return role == ClientRole::OBSERVER ? ClientRole::COMMANDER
                                        : ClientRole::OBSERVER;
  }
  // Thread-safe copy of a session's sink (empty when not connected).
  std::shared_ptr<UwsFrameSink> sinkOf(ClientSession& s);

  std::atomic<CameraState> system_state{CameraState::IDLE};

  // uWebSockets app
  uWS::App* app = nullptr;
  uWS::Loop* loop = nullptr;

  // Message handlers
  void handleDiscover(Socket* ws, const nlohmann::json& params);
  void handleGetState(Socket* ws);
  void handleConfigure(Socket* ws, const nlohmann::json& params);
  void handleUnconfigure(Socket* ws);
  void handleSetProcessMode(Socket* ws, const nlohmann::json& params);
  void handleTriggerCapture(Socket* ws, const nlohmann::json& msg);
  void handleStartCameras(Socket* ws);
  void handleStartStream(Socket* ws, const nlohmann::json& msg);
  void handleStopStream(Socket* ws, const nlohmann::json& msg);
  void handleStopCameras(Socket* ws);
  void handleResetFrameCounts(Socket* ws);
  void handleSetHeaderOnly(Socket* ws, const nlohmann::json& msg);
  void handleSetLensPosition(Socket* ws, const nlohmann::json& msg);
  void handleSetExposureTime(Socket* ws, const nlohmann::json& msg);
  void handleSetFrameDuration(Socket* ws, const nlohmann::json& msg);
  void handleGetFrameDurationLimits(Socket* ws);
  void handleGetLensPositionLimits(Socket* ws);

  void sendStatus(Socket* ws, const std::string& message);
  // `code` (optional) is the machine-readable `code` field of the error.
  void sendError(Socket* ws, const std::string& message,
                 const char* code = nullptr);

  // --- State reporting -----------------------------------------------------
  // The `state` message doubles as the reply to get_state (cause "query") and
  // as an unsolicited push on every transition and on the other role's
  // connect/disconnect, so an observer waiting for a commander to start the
  // cameras learns about it without polling.
  static const char* stateName(CameraState state);
  nlohmann::json stateMessage(ClientRole recipient, const char* cause);
  void pushStateTo(ClientRole role, const char* cause);
  void broadcastState(const char* cause);

  void cleanupConnection(ClientSession& s);

  // --- Capture watchdog ---------------------------------------------------
  // libcamera's RPi pipeline handler abandons a stalled frontend ("Camera
  // frontend has timed out!") and then never calls requestCompleted again.
  // Nothing surfaces that to the application, so without this the server sits
  // there with an open socket and a dead sensor while the client waits
  // forever. The watchdog turns that silence into an `error` message.
  void startCaptureWatchdog();
  void stopCaptureWatchdog();
  void captureWatchdogLoop();
  // Report one stalled camera to every connected client. Runs on the
  // watchdog thread, so it goes out through UwsFrameSink::sendText (which
  // marshals onto the uWS loop) rather than touching the sockets directly.
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
