#include "command_parser.hpp"

namespace command_parser {

using nlohmann::json;

std::variant<ParsedCommand, std::string> parseCommand(std::string_view raw) {
  json msg;
  try {
    msg = json::parse(raw);
  } catch (const json::exception& e) {
    return std::string("JSON parse error: ") + e.what();
  }

  if (!msg.contains("cmd") || !msg["cmd"].is_string()) {
    return std::string("Missing or non-string 'cmd' field");
  }

  std::string cmd = msg["cmd"].get<std::string>();
  CommandKind kind;
  if (cmd == Protocol::CMD_DISCOVER) {
    kind = CommandKind::Discover;
  } else if (cmd == Protocol::CMD_GET_STATE) {
    kind = CommandKind::GetState;
  } else if (cmd == Protocol::CMD_CONFIGURE) {
    kind = CommandKind::Configure;
  } else if (cmd == Protocol::CMD_UNCONFIGURE) {
    kind = CommandKind::Unconfigure;
  } else if (cmd == Protocol::CMD_SET_SAVE_MODE) {
    kind = CommandKind::SetSaveMode;
  } else if (cmd == Protocol::CMD_START_CAMERAS) {
    kind = CommandKind::StartCameras;
  } else if (cmd == Protocol::CMD_START_STREAM) {
    kind = CommandKind::StartStream;
  } else if (cmd == Protocol::CMD_STOP_STREAM) {
    kind = CommandKind::StopStream;
  } else if (cmd == Protocol::CMD_STOP_CAMERAS) {
    kind = CommandKind::StopCameras;
  } else if (cmd == Protocol::CMD_RESET_FRAME_COUNTS) {
    kind = CommandKind::ResetFrameCounts;
  } else if (cmd == Protocol::CMD_SET_HEADER_ONLY) {
    kind = CommandKind::SetHeaderOnly;
  } else if (cmd == Protocol::CMD_SET_LENS_POSITION) {
    kind = CommandKind::SetLensPosition;
  } else if (cmd == Protocol::CMD_SET_EXPOSURE_TIME) {
    kind = CommandKind::SetExposureTime;
  } else {
    return "Unknown command: " + cmd;
  }

  return ParsedCommand{kind, std::move(msg)};
}

std::optional<SaveMode> parseSaveMode(const std::string& mode) {
  if (mode == "none") return SaveMode::NONE;
  if (mode == "buffer") return SaveMode::BUFFER;
  if (mode == "batch") return SaveMode::BATCH;
  if (mode == "checkerboard") return SaveMode::CHECKERBOARD;
  return std::nullopt;
}

bool isCommandAllowed(CommandKind kind, CameraState current) {
  switch (kind) {
    case CommandKind::Discover:
    case CommandKind::GetState:
    case CommandKind::SetSaveMode:
    case CommandKind::ResetFrameCounts:
    case CommandKind::SetHeaderOnly:
      return true;
    case CommandKind::Configure:
      return current == CameraState::IDLE;
    case CommandKind::Unconfigure:
      return current == CameraState::CONFIGURED;
    case CommandKind::StartCameras:
      return current == CameraState::CONFIGURED;
    case CommandKind::StartStream:
    case CommandKind::StopStream:
    case CommandKind::StopCameras:
      return current == CameraState::RUNNING;
    case CommandKind::SetLensPosition:
    case CommandKind::SetExposureTime:
      return current == CameraState::CONFIGURED ||
             current == CameraState::RUNNING;
  }
  return false;
}

CameraConfig buildCameraConfig(const json& params) {
  CameraConfig config;
  if (params.contains("width")) config.width = params["width"];
  if (params.contains("height")) config.height = params["height"];
  if (params.contains("crop_width")) config.crop_width = params["crop_width"];
  if (params.contains("crop_height"))
    config.crop_height = params["crop_height"];
  if (params.contains("crop_left")) config.crop_left = params["crop_left"];
  if (params.contains("crop_top")) config.crop_top = params["crop_top"];

  // AWB fields are accepted but ignored (libcamera IPA owns AWB).
  if (params.contains("awb") && params["awb"].is_object()) {
    const json& a = params["awb"];
    if (a.contains("enabled")) config.awb.enabled = a["enabled"];
    if (a.contains("interval")) config.awb.interval = a["interval"];
    if (a.contains("speed")) config.awb.speed = a["speed"];
    if (a.contains("warmup_frames"))
      config.awb.warmup_frames = a["warmup_frames"];
  }
  return config;
}

std::optional<SaveConfig> buildSaveConfig(const json& message) {
  if (!message.contains("mode") || !message["mode"].is_string()) {
    return std::nullopt;
  }
  auto mode = parseSaveMode(message["mode"].get<std::string>());
  if (!mode) return std::nullopt;

  SaveConfig config;
  config.mode = *mode;

  if (message.contains("params")) {
    const json& params = message["params"];
    if (params.contains("output_dir")) config.output_dir = params["output_dir"];
    if (params.contains("prepend_timestamp_to_dir"))
      config.prepend_timestamp_to_dir = params["prepend_timestamp_to_dir"];
    if (params.contains("batch_size")) config.batch_size = params["batch_size"];
    if (params.contains("writer_threads"))
      config.writer_threads = params["writer_threads"];
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

  return config;
}

}  // namespace command_parser
