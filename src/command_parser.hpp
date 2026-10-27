#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include <nlohmann/json.hpp>

#include "raw_stacker.hpp"
#include "trigger_capturer.hpp"
#include "types.hpp"

// Pure parsing/validation for the WebSocket command protocol. No socket I/O;
// no system state. Easily unit-testable with synthetic JSON strings.
namespace command_parser {

enum class CommandKind {
  Discover,
  GetState,
  Configure,
  Unconfigure,
  SetProcessMode,
  TriggerCapture,
  StartCameras,
  StartStream,
  StopStream,
  StopCameras,
  ResetFrameCounts,
  SetHeaderOnly,
  SetLensPosition,
  SetExposureTime,
  SetFrameDuration,
  GetFrameDurationLimits,
  GetLensPositionLimits,
  GetFocusFomRegion,
  StackCapture,
};

struct ParsedCommand {
  CommandKind kind;
  // Copy of the original JSON message (commands with nested "params" or
  // sibling fields like "camera_id" differ by command, so the handler decides
  // what to read). Kept as nlohmann::json since the handlers already use it.
  nlohmann::json message;
};

// Parse a raw client message. Returns the parsed command on success, or an
// error string suitable for sending back to the client.
std::variant<ParsedCommand, std::string> parseCommand(std::string_view raw);

// Map the text "mode" in a set_process_mode message. Returns std::nullopt for an
// unknown mode (caller can surface an error).
std::optional<ProcessMode> parseProcessMode(const std::string& mode);

// Which role a connection is granted from the URL path it upgraded on:
// "/observer" (optionally with a trailing slash or query string) ⇒ OBSERVER;
// every other path — "/", "/commander", anything else — ⇒ COMMANDER, so
// pre-role clients that connect to the bare URL keep their read-write access.
ClientRole roleForPath(std::string_view path);

// Role gate: may a client holding `role` issue `kind` at all? A commander may
// issue anything. An observer is limited to the read-only set — `discover`,
// `get_state`, `start_stream`, `stop_stream`, `set_header_only`,
// `get_frame_duration_limits`, `get_lens_position_limits`,
// `get_focus_fom_region` — and everything
// else is refused with an error carrying `code: "forbidden"`.
bool isCommandPermitted(CommandKind kind, ClientRole role);

// State-machine gate: is `kind` allowed while the system is in `current`?
// `discover`, `set_process_mode`, `reset_frame_counts`, `set_header_only` are
// always allowed. Lifecycle commands gate on `current`; `trigger_capture` and
// `stack_capture` require RUNNING.
// `role` matters only for `start_stream` / `stop_stream`: a commander needs
// RUNNING, whereas an observer's subscription is persistent and may be made
// in any state (frames flow whenever the cameras are running).
bool isCommandAllowed(CommandKind kind, CameraState current,
                      ClientRole role = ClientRole::COMMANDER);

// Fields of a start_stream / stop_stream message.
struct StreamRequest {
  uint32_t camera_id = 0;
  StreamOptions options;
};

// Parse `camera_id` (required, non-negative integer) plus the optional
// start_stream options `max_fps` (number >= 0; 0 = uncapped) and `subsample`
// (1, 2, 4 or 8). Returns an error string suitable for the client on invalid
// input. Pure: camera existence is checked by the server.
std::variant<StreamRequest, std::string> buildStreamRequest(
    const nlohmann::json& message);

// Parse a stack_capture message: `camera_id` (optional), `num_frames`
// (1..1024, default 16), `exposure_time` (us, 1..10 000 000), `analogue_gain`
// (>= 1, <= 64; defaults to 1.0 when `exposure_time` is given), `skip_frames`
// (0..100), `send_image` (boolean, default true). The fields protocol v8
// removed — `detect`, `subpix_window`, `save_image`, `output_dir` — are
// refused rather than ignored, so a client relying on them learns they no
// longer do anything. Returns an error string suitable for the client on
// invalid input.
std::variant<StackRequest, std::string> buildStackRequest(
    const nlohmann::json& message);

// Parse a trigger_capture message: `camera_id` (optional), `skip_frames`
// (0..100, default 0), `send_image` (boolean, default true). Returns an error
// string suitable for the client on invalid input.
std::variant<TriggerRequest, std::string> buildTriggerRequest(
    const nlohmann::json& message);

// Convert a successful CameraConfig build. Pure: no camera, no transport.
// The function does not reject unknown fields — it silently ignores them
// (matches the server's existing behaviour).
CameraConfig buildCameraConfig(const nlohmann::json& params);

// Extract the "sensor" substring filter from a discover message's "params".
// Defaults to "imx519" when absent or not a string (matches the server's
// pre-existing hardcoded behaviour).
std::string buildSensorFilter(const nlohmann::json& params);

// The sensor filter a discover should enumerate with, given the filter of the
// cameras already discovered (`active_sensor`, lower-case; empty if none), or
// std::nullopt to keep those cameras. With none discovered it is
// buildSensorFilter's. After that only a `sensor` that differs
// (case-insensitively) switches: an absent one keeps the cameras, so a client
// that names no sensor never tears down another's.
std::optional<std::string> sensorToDiscover(const nlohmann::json& params,
                                            const std::string& active_sensor);

// Convert a successful ProcessConfig build for a set_process_mode message. Returns
// std::nullopt if the "mode" field is missing or unknown.
std::optional<ProcessConfig> buildProcessConfig(const nlohmann::json& message);

}  // namespace command_parser
