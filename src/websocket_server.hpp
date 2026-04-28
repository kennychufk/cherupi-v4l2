#pragma once

#include <atomic>
#include <memory>

#include "camera_manager.hpp"
#include "frame_saver.hpp"
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
  std::unique_ptr<FrameSaver> frame_saver;
  std::unique_ptr<StreamManager> stream_manager;
  std::shared_ptr<UwsFrameSink> active_sink;

  std::atomic<bool> has_client{false};
  std::atomic<CameraState> system_state{CameraState::IDLE};

  // uWebSockets app
  uWS::App* app = nullptr;
  uWS::Loop* loop = nullptr;

  // Message handlers
  void handleDiscover(uWS::WebSocket<false, true, int>* ws);
  void handleConfigure(uWS::WebSocket<false, true, int>* ws,
                       const nlohmann::json& params);
  void handleUnconfigure(uWS::WebSocket<false, true, int>* ws);
  void handleSetSaveMode(uWS::WebSocket<false, true, int>* ws,
                         const nlohmann::json& params);
  void handleStartCameras(uWS::WebSocket<false, true, int>* ws);
  void handleStartStream(uWS::WebSocket<false, true, int>* ws,
                         const nlohmann::json& params);
  void handleStopStream(uWS::WebSocket<false, true, int>* ws,
                        const nlohmann::json& params);
  void handleStopCameras(uWS::WebSocket<false, true, int>* ws);
  void handleResetFrameCounts(uWS::WebSocket<false, true, int>* ws);
  void handleSetHeaderOnly(uWS::WebSocket<false, true, int>* ws,
                           const nlohmann::json& msg);

  void sendStatus(uWS::WebSocket<false, true, int>* ws,
                  const std::string& message);
  void sendError(uWS::WebSocket<false, true, int>* ws,
                 const std::string& message);

  void cleanupConnection();

 public:
  WebSocketServer();
  ~WebSocketServer();

  void run();
  void stop();
};
