#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include <nlohmann/json.hpp>

#include "types.hpp"

// Pure parsing/validation for the WebSocket command protocol. No socket I/O;
// no system state. Easily unit-testable with synthetic JSON strings.
namespace command_parser {

enum class CommandKind {
  Discover,
  GetState,
  Configure,
  Unconfigure,
  SetSaveMode,
  StartCameras,
  StartStream,
  StopStream,
  StopCameras,
  ResetFrameCounts,
  SetHeaderOnly,
  SetLensPosition,
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

// Map the text "mode" in a set_save_mode message. Returns std::nullopt for an
// unknown mode (caller can surface an error).
std::optional<SaveMode> parseSaveMode(const std::string& mode);

// State-machine gate: is `kind` allowed while the system is in `current`?
// `discover`, `set_save_mode`, `reset_frame_counts`, `set_header_only` are
// always allowed. Lifecycle commands gate on `current`.
bool isCommandAllowed(CommandKind kind, CameraState current);

// Convert a successful CameraConfig build. Pure: no camera, no transport.
// The function does not reject unknown fields — it silently ignores them
// (matches the server's existing behaviour). AWB fields are accepted but
// discarded since the libcamera IPA owns AWB now (see CLAUDE.md).
CameraConfig buildCameraConfig(const nlohmann::json& params);

// Convert a successful SaveConfig build for a set_save_mode message. Returns
// std::nullopt if the "mode" field is missing or unknown.
std::optional<SaveConfig> buildSaveConfig(const nlohmann::json& message);

}  // namespace command_parser
