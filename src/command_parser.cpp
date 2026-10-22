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
  } else if (cmd == Protocol::CMD_SET_PROCESS_MODE) {
    kind = CommandKind::SetProcessMode;
  } else if (cmd == Protocol::CMD_TRIGGER_CAPTURE) {
    kind = CommandKind::TriggerCapture;
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
  } else if (cmd == Protocol::CMD_SET_FRAME_DURATION) {
    kind = CommandKind::SetFrameDuration;
  } else if (cmd == Protocol::CMD_GET_FRAME_DURATION_LIMITS) {
    kind = CommandKind::GetFrameDurationLimits;
  } else if (cmd == Protocol::CMD_GET_LENS_POSITION_LIMITS) {
    kind = CommandKind::GetLensPositionLimits;
  } else if (cmd == Protocol::CMD_GET_FOCUS_FOM_REGION) {
    kind = CommandKind::GetFocusFomRegion;
  } else if (cmd == Protocol::CMD_STACK_CAPTURE) {
    kind = CommandKind::StackCapture;
  } else {
    return "Unknown command: " + cmd;
  }

  return ParsedCommand{kind, std::move(msg)};
}

std::optional<ProcessMode> parseProcessMode(const std::string& mode) {
  if (mode == "none") return ProcessMode::NONE;
  if (mode == "buffer") return ProcessMode::BUFFER;
  if (mode == "batch") return ProcessMode::BATCH;
  if (mode == "checkerboard") return ProcessMode::CHECKERBOARD;
  if (mode == "checkerboard2x2") return ProcessMode::CHECKERBOARD2X2;
  if (mode == "aruco") return ProcessMode::ARUCO;
  if (mode == "aruco2x2") return ProcessMode::ARUCO2X2;
  return std::nullopt;
}

ClientRole roleForPath(std::string_view path) {
  // Drop any query string, then a single trailing slash.
  const size_t q = path.find('?');
  if (q != std::string_view::npos) path = path.substr(0, q);
  if (path.size() > 1 && path.back() == '/') path.remove_suffix(1);
  return path == "/observer" ? ClientRole::OBSERVER : ClientRole::COMMANDER;
}

bool isCommandPermitted(CommandKind kind, ClientRole role) {
  if (role == ClientRole::COMMANDER) return true;
  switch (kind) {
    case CommandKind::Discover:
    case CommandKind::GetState:
    case CommandKind::StartStream:
    case CommandKind::StopStream:
    case CommandKind::SetHeaderOnly:
    case CommandKind::GetFrameDurationLimits:
    case CommandKind::GetLensPositionLimits:
    case CommandKind::GetFocusFomRegion:
      return true;
    case CommandKind::Configure:
    case CommandKind::Unconfigure:
    case CommandKind::SetProcessMode:
    case CommandKind::TriggerCapture:
    case CommandKind::StartCameras:
    case CommandKind::StopCameras:
    case CommandKind::ResetFrameCounts:
    case CommandKind::SetLensPosition:
    case CommandKind::SetExposureTime:
    case CommandKind::SetFrameDuration:
    case CommandKind::StackCapture:
      return false;
  }
  return false;
}

bool isCommandAllowed(CommandKind kind, CameraState current, ClientRole role) {
  switch (kind) {
    case CommandKind::Discover:
    case CommandKind::GetState:
    case CommandKind::SetProcessMode:
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
      // An observer subscribes once and keeps the subscription across camera
      // stop/start cycles, so it may do so whatever the current state.
      return role == ClientRole::OBSERVER || current == CameraState::RUNNING;
    case CommandKind::StopCameras:
    case CommandKind::TriggerCapture:
    case CommandKind::StackCapture:
      return current == CameraState::RUNNING;
    case CommandKind::SetLensPosition:
    case CommandKind::SetExposureTime:
    case CommandKind::SetFrameDuration:
    case CommandKind::GetFrameDurationLimits:
    case CommandKind::GetLensPositionLimits:
    case CommandKind::GetFocusFomRegion:
      return current == CameraState::CONFIGURED ||
             current == CameraState::RUNNING;
  }
  return false;
}

CameraConfig buildCameraConfig(const json& params) {
  CameraConfig config;
  if (params.contains("width")) config.width = params["width"];
  if (params.contains("height")) config.height = params["height"];
  if (params.contains("raw") && params["raw"].is_boolean())
    config.raw = params["raw"];
  return config;
}

std::string buildSensorFilter(const json& params) {
  if (params.contains("sensor") && params["sensor"].is_string()) {
    return params["sensor"].get<std::string>();
  }
  return "imx519";
}

std::variant<StreamRequest, std::string> buildStreamRequest(
    const json& message) {
  if (!message.contains("camera_id") ||
      !message["camera_id"].is_number_integer()) {
    return std::string("Missing or non-integer 'camera_id' field");
  }
  const int64_t requested = message["camera_id"].get<int64_t>();
  if (requested < 0) return std::string("camera_id must be non-negative");

  StreamRequest req;
  req.camera_id = static_cast<uint32_t>(requested);

  if (message.contains("max_fps")) {
    const json& v = message["max_fps"];
    if (!v.is_number()) return std::string("max_fps must be a number");
    const double fps = v.get<double>();
    if (!(fps >= 0.0) || fps > 1000.0) {
      return std::string("max_fps out of range (expected 0 for uncapped, or (0, 1000])");
    }
    req.options.max_fps = fps;
  }

  if (message.contains("subsample")) {
    const json& v = message["subsample"];
    if (!v.is_number_integer() || v.get<int64_t>() < 1 ||
        v.get<int64_t>() > 8) {
      return std::string("subsample must be 1, 2, 4 or 8");
    }
    const uint32_t factor = static_cast<uint32_t>(v.get<int64_t>());
    if (factor != 1 && factor != 2 && factor != 4 && factor != 8) {
      return std::string("subsample must be 1, 2, 4 or 8");
    }
    req.options.subsample = factor;
  }

  return req;
}

std::optional<ProcessConfig> buildProcessConfig(const json& message) {
  if (!message.contains("mode") || !message["mode"].is_string()) {
    return std::nullopt;
  }
  auto mode = parseProcessMode(message["mode"].get<std::string>());
  if (!mode) return std::nullopt;

  ProcessConfig config;
  config.mode = *mode;

  if (message.contains("params")) {
    const json& params = message["params"];
    if (params.contains("save_frames") && params["save_frames"].is_boolean())
      config.save_frames = params["save_frames"];
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
    if (params.contains("checkerboard_subpix_window"))
      config.checkerboard_subpix_window = params["checkerboard_subpix_window"];
    if (params.contains("aruco_full_res_detection"))
      config.aruco_full_res_detection = params["aruco_full_res_detection"];
    if (params.contains("aruco_num_threads"))
      config.aruco_num_threads = params["aruco_num_threads"];
    if (params.contains("aruco_corner_refine"))
      config.aruco_corner_refine = params["aruco_corner_refine"];
    if (params.contains("aruco_edge_refine"))
      config.aruco_edge_refine = params["aruco_edge_refine"];
    if (params.contains("aruco_edge_gamma") &&
        params["aruco_edge_gamma"].is_number()) {
      // A non-positive exponent cannot decode anything; treat it as "none".
      const double g = params["aruco_edge_gamma"].get<double>();
      config.aruco_edge_gamma = g > 0.0 ? static_cast<float>(g) : 1.0f;
    }
    // Resource guards. All three are optional; 0 / absent means "decide at
    // start()" for the two byte counts, and "refuse an infeasible mode" for
    // allow_overcommit.
    if (params.contains("backlog_max_bytes") &&
        params["backlog_max_bytes"].is_number_unsigned())
      config.backlog_max_bytes = params["backlog_max_bytes"].get<size_t>();
    if (params.contains("disk_write_bytes_per_sec") &&
        params["disk_write_bytes_per_sec"].is_number_unsigned())
      config.disk_write_bytes_per_sec =
          params["disk_write_bytes_per_sec"].get<size_t>();
    if (params.contains("allow_overcommit") &&
        params["allow_overcommit"].is_boolean())
      config.allow_overcommit = params["allow_overcommit"];
  }

  return config;
}

namespace {

// Range-checked integer field of `message`: nothing when absent, an error
// naming the field when present but not an integer in [lo, hi].
std::optional<std::string> integerField(const json& message, const char* key,
                                        int64_t lo, int64_t hi, int64_t* out) {
  if (!message.contains(key)) return std::nullopt;
  const json& v = message[key];
  if (!v.is_number_integer() || v.get<int64_t>() < lo || v.get<int64_t>() > hi)
    return std::string(key) + " must be an integer in [" + std::to_string(lo) +
           ", " + std::to_string(hi) + "]";
  *out = v.get<int64_t>();
  return std::nullopt;
}

// `send_image` (default true): whether a capture's frames carry their pixels.
std::optional<std::string> sendImageField(const json& message, bool* out) {
  if (!message.contains("send_image")) return std::nullopt;
  if (!message["send_image"].is_boolean())
    return std::string("send_image must be a boolean");
  *out = message["send_image"];
  return std::nullopt;
}

}  // namespace

std::variant<StackRequest, std::string> buildStackRequest(const json& message) {
  StackRequest req;
  int64_t v = 0;
  if (auto err = integerField(message, "camera_id", 0, 255, &v)) return *err;
  if (message.contains("camera_id")) req.camera_id = static_cast<uint32_t>(v);
  if (auto err = integerField(message, "num_frames", 1,
                              raw_stack::Accumulator::kMaxFrames, &v))
    return *err;
  if (message.contains("num_frames")) req.num_frames = static_cast<uint32_t>(v);
  if (auto err = integerField(message, "exposure_time", 1, 10000000, &v))
    return *err;
  if (message.contains("exposure_time"))
    req.exposure_time_us = static_cast<int32_t>(v);
  if (message.contains("analogue_gain")) {
    const json& g = message["analogue_gain"];
    if (!g.is_number() || !(g.get<double>() >= 1.0) || g.get<double>() > 64.0)
      return std::string("analogue_gain must be a number in [1, 64]");
    req.analogue_gain = static_cast<float>(g.get<double>());
  } else if (req.exposure_time_us) {
    req.analogue_gain = 1.0f;
  }
  if (auto err = integerField(message, "skip_frames", 0, 100, &v)) return *err;
  if (message.contains("skip_frames")) req.skip_frames = static_cast<int>(v);
  if (auto err = sendImageField(message, &req.send_image)) return *err;
  return req;
}

std::variant<TriggerRequest, std::string> buildTriggerRequest(
    const json& message) {
  TriggerRequest req;
  int64_t v = 0;
  if (auto err = integerField(message, "camera_id", 0, 255, &v)) return *err;
  if (message.contains("camera_id")) req.camera_id = static_cast<uint32_t>(v);
  if (auto err = integerField(message, "skip_frames", 0, 100, &v)) return *err;
  if (message.contains("skip_frames")) req.skip_frames = static_cast<int>(v);
  if (auto err = sendImageField(message, &req.send_image)) return *err;
  return req;
}

}  // namespace command_parser
