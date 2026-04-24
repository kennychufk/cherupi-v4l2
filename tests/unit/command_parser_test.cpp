#include <gtest/gtest.h>

#include <variant>

#include "command_parser.hpp"

namespace {

using command_parser::CommandKind;
using command_parser::ParsedCommand;

TEST(CommandParserTest, ValidDiscoverCommand) {
  auto r = command_parser::parseCommand(R"({"cmd":"discover"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::Discover);
}

TEST(CommandParserTest, ValidConfigureCommand) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"configure","params":{"width":640,"height":480}})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  const auto& cmd = std::get<ParsedCommand>(r);
  EXPECT_EQ(cmd.kind, CommandKind::Configure);
  EXPECT_EQ(cmd.message["params"]["width"], 640);
  EXPECT_EQ(cmd.message["params"]["height"], 480);
}

TEST(CommandParserTest, InvalidJsonReturnsError) {
  auto r = command_parser::parseCommand("{ not json");
  ASSERT_TRUE(std::holds_alternative<std::string>(r));
  EXPECT_NE(std::get<std::string>(r).find("JSON parse error"),
            std::string::npos);
}

TEST(CommandParserTest, MissingCmdFieldReturnsError) {
  auto r = command_parser::parseCommand(R"({"params":{}})");
  ASSERT_TRUE(std::holds_alternative<std::string>(r));
  EXPECT_NE(std::get<std::string>(r).find("cmd"), std::string::npos);
}

TEST(CommandParserTest, UnknownCommandReturnsError) {
  auto r = command_parser::parseCommand(R"({"cmd":"nuke_everything"})");
  ASSERT_TRUE(std::holds_alternative<std::string>(r));
  EXPECT_NE(std::get<std::string>(r).find("nuke_everything"),
            std::string::npos);
}

TEST(CommandParserTest, SaveModeLookup) {
  EXPECT_EQ(command_parser::parseSaveMode("none"), SaveMode::NONE);
  EXPECT_EQ(command_parser::parseSaveMode("buffer"), SaveMode::BUFFER);
  EXPECT_EQ(command_parser::parseSaveMode("batch"), SaveMode::BATCH);
  EXPECT_EQ(command_parser::parseSaveMode("checkerboard"),
            SaveMode::CHECKERBOARD);
  EXPECT_EQ(command_parser::parseSaveMode("bogus"), std::nullopt);
}

TEST(CommandParserTest, StateGateAllowsAlwaysAllowedCommands) {
  for (auto state : {CameraState::IDLE, CameraState::CONFIGURED,
                     CameraState::RUNNING, CameraState::ERROR}) {
    EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::Discover, state));
    EXPECT_TRUE(
        command_parser::isCommandAllowed(CommandKind::SetSaveMode, state));
    EXPECT_TRUE(
        command_parser::isCommandAllowed(CommandKind::ResetFrameCounts, state));
    EXPECT_TRUE(
        command_parser::isCommandAllowed(CommandKind::SetHeaderOnly, state));
  }
}

TEST(CommandParserTest, StateGateConfigureRequiresIdle) {
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::Configure,
                                               CameraState::IDLE));
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::Configure,
                                                CameraState::CONFIGURED));
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::Configure,
                                                CameraState::RUNNING));
}

TEST(CommandParserTest, StateGateStartCamerasRequiresConfigured) {
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::StartCameras,
                                                CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::StartCameras,
                                               CameraState::CONFIGURED));
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::StartCameras,
                                                CameraState::RUNNING));
}

TEST(CommandParserTest, StateGateStreamCommandsRequireRunning) {
  for (auto cmd : {CommandKind::StartStream, CommandKind::StopStream,
                   CommandKind::StopCameras}) {
    EXPECT_FALSE(command_parser::isCommandAllowed(cmd, CameraState::IDLE));
    EXPECT_FALSE(
        command_parser::isCommandAllowed(cmd, CameraState::CONFIGURED));
    EXPECT_TRUE(command_parser::isCommandAllowed(cmd, CameraState::RUNNING));
  }
}

TEST(CommandParserTest, BuildCameraConfigOverridesDefaults) {
  auto params = nlohmann::json::parse(
      R"({"width":1280,"height":720,"crop_left":100,"crop_top":50})");
  CameraConfig cfg = command_parser::buildCameraConfig(params);
  EXPECT_EQ(cfg.width, 1280u);
  EXPECT_EQ(cfg.height, 720u);
  EXPECT_EQ(cfg.crop_left, 100u);
  EXPECT_EQ(cfg.crop_top, 50u);
  // Un-specified fields keep their defaults.
  EXPECT_EQ(cfg.crop_width, CameraConfig{}.crop_width);
}

TEST(CommandParserTest, BuildCameraConfigAwbFieldsAcceptedButRetained) {
  // AWB fields are documented as "accepted-but-ignored" by the runtime (the
  // libcamera IPA owns AWB). The parser still parses them into the struct so
  // the protocol contract is faithfully represented; the runtime drops them.
  auto params = nlohmann::json::parse(
      R"({"awb":{"enabled":false,"interval":33,"speed":0.1,"warmup_frames":20}})");
  CameraConfig cfg = command_parser::buildCameraConfig(params);
  EXPECT_FALSE(cfg.awb.enabled);
  EXPECT_EQ(cfg.awb.interval, 33);
  EXPECT_FLOAT_EQ(cfg.awb.speed, 0.1f);
  EXPECT_EQ(cfg.awb.warmup_frames, 20);
}

TEST(CommandParserTest, BuildSaveConfigAppliesMode) {
  auto msg = nlohmann::json::parse(R"({"mode":"batch","params":{"batch_size":7}})");
  auto cfg = command_parser::buildSaveConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->mode, SaveMode::BATCH);
  EXPECT_EQ(cfg->batch_size, 7u);
}

TEST(CommandParserTest, BuildSaveConfigRejectsUnknownMode) {
  auto msg = nlohmann::json::parse(R"({"mode":"telepathy"})");
  auto cfg = command_parser::buildSaveConfig(msg);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CommandParserTest, BuildSaveConfigRejectsMissingMode) {
  auto msg = nlohmann::json::parse(R"({"params":{}})");
  auto cfg = command_parser::buildSaveConfig(msg);
  EXPECT_FALSE(cfg.has_value());
}

}  // namespace
