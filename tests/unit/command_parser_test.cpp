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

TEST(CommandParserTest, ProcessModeLookup) {
  EXPECT_EQ(command_parser::parseProcessMode("none"), ProcessMode::NONE);
  EXPECT_EQ(command_parser::parseProcessMode("buffer"), ProcessMode::BUFFER);
  EXPECT_EQ(command_parser::parseProcessMode("batch"), ProcessMode::BATCH);
  EXPECT_EQ(command_parser::parseProcessMode("checkerboard"),
            ProcessMode::CHECKERBOARD);
  EXPECT_EQ(command_parser::parseProcessMode("checkerboard2x2"),
            ProcessMode::CHECKERBOARD2X2);
  EXPECT_EQ(command_parser::parseProcessMode("aruco"), ProcessMode::ARUCO);
  EXPECT_EQ(command_parser::parseProcessMode("aruco2x2"), ProcessMode::ARUCO2X2);
  EXPECT_EQ(command_parser::parseProcessMode("bogus"), std::nullopt);
  // On-demand captures are commands, not modes (protocol v8).
  EXPECT_EQ(command_parser::parseProcessMode("trigger"), std::nullopt);
  EXPECT_EQ(command_parser::parseProcessMode("stack"), std::nullopt);
}

TEST(CommandParserTest, StateGateAllowsAlwaysAllowedCommands) {
  for (auto state : {CameraState::IDLE, CameraState::CONFIGURED,
                     CameraState::RUNNING, CameraState::ERROR}) {
    EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::Discover, state));
    EXPECT_TRUE(
        command_parser::isCommandAllowed(CommandKind::SetProcessMode, state));
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

TEST(CommandParserTest, ValidUnconfigureCommand) {
  auto r = command_parser::parseCommand(R"({"cmd":"unconfigure"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::Unconfigure);
}

TEST(CommandParserTest, StateGateUnconfigureRequiresConfigured) {
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::Unconfigure,
                                                CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::Unconfigure,
                                               CameraState::CONFIGURED));
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::Unconfigure,
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
                   CommandKind::StopCameras, CommandKind::TriggerCapture}) {
    EXPECT_FALSE(command_parser::isCommandAllowed(cmd, CameraState::IDLE));
    EXPECT_FALSE(
        command_parser::isCommandAllowed(cmd, CameraState::CONFIGURED));
    EXPECT_TRUE(command_parser::isCommandAllowed(cmd, CameraState::RUNNING));
  }
}

TEST(CommandParserTest, BuildCameraConfigOverridesDefaults) {
  auto params = nlohmann::json::parse(R"({"width":1280})");
  CameraConfig cfg = command_parser::buildCameraConfig(params);
  EXPECT_EQ(cfg.width, 1280u);
  // Un-specified fields keep their defaults.
  EXPECT_EQ(cfg.height, CameraConfig{}.height);
}

TEST(CommandParserTest, BuildSensorFilterDefaultsToImx519) {
  auto params = nlohmann::json::object();
  EXPECT_EQ(command_parser::buildSensorFilter(params), "imx519");
}

TEST(CommandParserTest, BuildSensorFilterUsesProvidedSensor) {
  auto params = nlohmann::json::parse(R"({"sensor":"imx477"})");
  EXPECT_EQ(command_parser::buildSensorFilter(params), "imx477");
}

TEST(CommandParserTest, BuildSensorFilterIgnoresNonStringSensor) {
  auto params = nlohmann::json::parse(R"({"sensor":123})");
  EXPECT_EQ(command_parser::buildSensorFilter(params), "imx519");
}

TEST(CommandParserTest, BuildProcessConfigAppliesMode) {
  auto msg = nlohmann::json::parse(R"({"mode":"batch","params":{"batch_size":7}})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->mode, ProcessMode::BATCH);
  EXPECT_EQ(cfg->batch_size, 7u);
}

TEST(CommandParserTest, ValidTriggerCaptureCommand) {
  auto result = command_parser::parseCommand(R"({"cmd":"trigger_capture"})");
  ASSERT_TRUE(std::holds_alternative<command_parser::ParsedCommand>(result));
  EXPECT_EQ(std::get<command_parser::ParsedCommand>(result).kind,
            CommandKind::TriggerCapture);
}

TEST(CommandParserTest, BuildTriggerRequestDefaultsAndFields) {
  auto d = command_parser::buildTriggerRequest(
      nlohmann::json::parse(R"({"cmd":"trigger_capture"})"));
  ASSERT_TRUE(std::holds_alternative<TriggerRequest>(d));
  EXPECT_FALSE(std::get<TriggerRequest>(d).camera_id);
  EXPECT_EQ(std::get<TriggerRequest>(d).skip_frames, 0);
  EXPECT_TRUE(std::get<TriggerRequest>(d).send_image);

  auto r = command_parser::buildTriggerRequest(nlohmann::json::parse(
      R"({"camera_id":2,"skip_frames":3,"send_image":false})"));
  ASSERT_TRUE(std::holds_alternative<TriggerRequest>(r));
  const auto& req = std::get<TriggerRequest>(r);
  EXPECT_EQ(req.camera_id, 2u);
  EXPECT_EQ(req.skip_frames, 3);
  EXPECT_FALSE(req.send_image);

  for (const char* bad : {R"({"camera_id":-1})", R"({"camera_id":"0"})",
                          R"({"skip_frames":-1})", R"({"skip_frames":101})",
                          R"({"send_image":1})"}) {
    EXPECT_TRUE(std::holds_alternative<std::string>(
        command_parser::buildTriggerRequest(nlohmann::json::parse(bad))))
        << bad;
  }
}

TEST(CommandParserTest, BuildProcessConfigAppliesArucoParams) {
  auto msg = nlohmann::json::parse(R"({
    "mode":"aruco2x2",
    "params":{
      "aruco_full_res_detection":true,
      "aruco_num_threads":2,
      "aruco_corner_refine":true,
      "aruco_edge_refine":false,
      "aruco_edge_gamma":1.8
    }
  })");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->mode, ProcessMode::ARUCO2X2);
  EXPECT_TRUE(cfg->aruco_full_res_detection);
  EXPECT_EQ(cfg->aruco_num_threads, 2);
  EXPECT_TRUE(cfg->aruco_corner_refine);
  EXPECT_FALSE(cfg->aruco_edge_refine);
  EXPECT_FLOAT_EQ(cfg->aruco_edge_gamma, 1.8f);
}

TEST(CommandParserTest, BuildProcessConfigArucoEdgeGammaNonPositiveMeansNone) {
  auto msg = nlohmann::json::parse(
      R"({"mode":"aruco","params":{"aruco_edge_gamma":0}})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_TRUE(cfg->aruco_edge_refine);
  EXPECT_FLOAT_EQ(cfg->aruco_edge_gamma, 1.0f);
}

TEST(CommandParserTest, BuildProcessConfigArucoDefaults) {
  auto msg = nlohmann::json::parse(R"({"mode":"aruco"})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->mode, ProcessMode::ARUCO);
  // Unspecified params keep their defaults.
  EXPECT_FALSE(cfg->aruco_full_res_detection);
  EXPECT_EQ(cfg->aruco_num_threads, 4);
  EXPECT_FALSE(cfg->aruco_corner_refine);
  // The edge refinement is on unless a client turns it off.
  EXPECT_TRUE(cfg->aruco_edge_refine);
  EXPECT_FLOAT_EQ(cfg->aruco_edge_gamma, 2.2f);
}

TEST(CommandParserTest, BuildProcessConfigRejectsUnknownMode) {
  auto msg = nlohmann::json::parse(R"({"mode":"telepathy"})");
  auto cfg = command_parser::buildProcessConfig(msg);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CommandParserTest, BuildProcessConfigRejectsMissingMode) {
  auto msg = nlohmann::json::parse(R"({"params":{}})");
  auto cfg = command_parser::buildProcessConfig(msg);
  EXPECT_FALSE(cfg.has_value());
}

TEST(CommandParserTest, BuildProcessConfigSaveFramesDefaultsTrue) {
  // Absent save_frames keeps the default (persist), preserving old behaviour.
  auto msg = nlohmann::json::parse(R"({"mode":"aruco"})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_TRUE(cfg->save_frames);
}

TEST(CommandParserTest, BuildProcessConfigParsesSaveFramesFalse) {
  auto msg = nlohmann::json::parse(
      R"({"mode":"aruco","params":{"save_frames":false}})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_FALSE(cfg->save_frames);
}

TEST(CommandParserTest, BuildProcessConfigIgnoresNonBoolSaveFrames) {
  // A non-boolean save_frames is ignored, leaving the default in place.
  auto msg = nlohmann::json::parse(
      R"({"mode":"aruco","params":{"save_frames":"nope"}})");
  auto cfg = command_parser::buildProcessConfig(msg);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_TRUE(cfg->save_frames);
}

TEST(CommandParserTest, ValidSetLensPositionManual) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_lens_position","lens_position":4.5})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  const auto& cmd = std::get<ParsedCommand>(r);
  EXPECT_EQ(cmd.kind, CommandKind::SetLensPosition);
  EXPECT_FLOAT_EQ(cmd.message["lens_position"].get<float>(), 4.5f);
}

TEST(CommandParserTest, ValidSetLensPositionAfSentinel) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_lens_position","lens_position":-1})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::SetLensPosition);
}

TEST(CommandParserTest, StateGateSetLensPositionRequiresConfiguredOrRunning) {
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::SetLensPosition,
                                                CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::SetLensPosition,
                                               CameraState::CONFIGURED));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::SetLensPosition,
                                               CameraState::RUNNING));
}

TEST(CommandParserTest, ValidSetExposureTimeManual) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_exposure_time","exposure_time":10000})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  const auto& cmd = std::get<ParsedCommand>(r);
  EXPECT_EQ(cmd.kind, CommandKind::SetExposureTime);
  EXPECT_EQ(cmd.message["exposure_time"].get<int32_t>(), 10000);
}

TEST(CommandParserTest, ValidSetExposureTimeAutoSentinel) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_exposure_time","exposure_time":-1})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::SetExposureTime);
}

TEST(CommandParserTest, StateGateSetExposureTimeRequiresConfiguredOrRunning) {
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::SetExposureTime,
                                                CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::SetExposureTime,
                                               CameraState::CONFIGURED));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::SetExposureTime,
                                               CameraState::RUNNING));
}

TEST(CommandParserTest, ValidSetFrameDurationLock) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_frame_duration","frame_duration":33333})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  const auto& cmd = std::get<ParsedCommand>(r);
  EXPECT_EQ(cmd.kind, CommandKind::SetFrameDuration);
  EXPECT_EQ(cmd.message["frame_duration"].get<int64_t>(), 33333);
}

TEST(CommandParserTest, ValidSetFrameDurationUnsetSentinel) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"set_frame_duration","frame_duration":-1})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::SetFrameDuration);
}

TEST(CommandParserTest, ValidGetFrameDurationLimits) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"get_frame_duration_limits"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind,
            CommandKind::GetFrameDurationLimits);
}

TEST(CommandParserTest, StateGateFrameDurationCmdsRequireConfiguredOrRunning) {
  for (auto kind : {CommandKind::SetFrameDuration,
                    CommandKind::GetFrameDurationLimits}) {
    EXPECT_FALSE(command_parser::isCommandAllowed(kind, CameraState::IDLE));
    EXPECT_TRUE(
        command_parser::isCommandAllowed(kind, CameraState::CONFIGURED));
    EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::RUNNING));
  }
}

TEST(CommandParserTest, ValidGetLensPositionLimits) {
  auto r = command_parser::parseCommand(
      R"({"cmd":"get_lens_position_limits"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind,
            CommandKind::GetLensPositionLimits);
}

TEST(CommandParserTest, StateGateGetLensPositionLimitsRequiresConfiguredOrRunning) {
  auto kind = CommandKind::GetLensPositionLimits;
  EXPECT_FALSE(command_parser::isCommandAllowed(kind, CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::CONFIGURED));
  EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::RUNNING));
}

TEST(CommandParserTest, ValidGetFocusFomRegion) {
  auto r = command_parser::parseCommand(R"({"cmd":"get_focus_fom_region"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::GetFocusFomRegion);
}

TEST(CommandParserTest, StateGateGetFocusFomRegionRequiresConfiguredOrRunning) {
  // The region depends on the configured sensor mode, so IDLE has none.
  auto kind = CommandKind::GetFocusFomRegion;
  EXPECT_FALSE(command_parser::isCommandAllowed(kind, CameraState::IDLE));
  EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::CONFIGURED));
  EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::RUNNING));
}

// --- Client roles ------------------------------------------------------------

TEST(CommandParserTest, RoleForPathSelectsObserverOnlyForObserverPath) {
  using command_parser::roleForPath;
  EXPECT_EQ(roleForPath("/"), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath(""), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath("/commander"), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath("/anything/else"), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath("/observer"), ClientRole::OBSERVER);
  EXPECT_EQ(roleForPath("/observer/"), ClientRole::OBSERVER);
  EXPECT_EQ(roleForPath("/observer?x=1"), ClientRole::OBSERVER);
  // Exact match only: no prefixes, no case folding.
  EXPECT_EQ(roleForPath("/observers"), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath("/observer/x"), ClientRole::COMMANDER);
  EXPECT_EQ(roleForPath("/Observer"), ClientRole::COMMANDER);
}

TEST(CommandParserTest, CommanderMayIssueEveryCommand) {
  for (CommandKind kind :
       {CommandKind::Discover, CommandKind::GetState, CommandKind::Configure,
        CommandKind::Unconfigure, CommandKind::SetProcessMode,
        CommandKind::TriggerCapture, CommandKind::StartCameras,
        CommandKind::StartStream, CommandKind::StopStream,
        CommandKind::StopCameras, CommandKind::ResetFrameCounts,
        CommandKind::SetHeaderOnly, CommandKind::SetLensPosition,
        CommandKind::SetExposureTime, CommandKind::SetFrameDuration,
        CommandKind::GetFrameDurationLimits,
        CommandKind::GetLensPositionLimits, CommandKind::GetFocusFomRegion}) {
    EXPECT_TRUE(command_parser::isCommandPermitted(kind, ClientRole::COMMANDER))
        << static_cast<int>(kind);
  }
}

TEST(CommandParserTest, ObserverIsLimitedToReadOnlyCommands) {
  for (CommandKind kind :
       {CommandKind::Discover, CommandKind::GetState, CommandKind::StartStream,
        CommandKind::StopStream, CommandKind::SetHeaderOnly,
        CommandKind::GetFrameDurationLimits,
        CommandKind::GetLensPositionLimits, CommandKind::GetFocusFomRegion}) {
    EXPECT_TRUE(command_parser::isCommandPermitted(kind, ClientRole::OBSERVER))
        << static_cast<int>(kind);
  }
  for (CommandKind kind :
       {CommandKind::Configure, CommandKind::Unconfigure,
        CommandKind::SetProcessMode, CommandKind::TriggerCapture,
        CommandKind::StartCameras, CommandKind::StopCameras,
        CommandKind::ResetFrameCounts, CommandKind::SetLensPosition,
        CommandKind::SetExposureTime, CommandKind::SetFrameDuration}) {
    EXPECT_FALSE(command_parser::isCommandPermitted(kind, ClientRole::OBSERVER))
        << static_cast<int>(kind);
  }
}

TEST(CommandParserTest, ObserverMaySubscribeInAnyStateCommanderNeedsRunning) {
  for (CommandKind kind : {CommandKind::StartStream, CommandKind::StopStream}) {
    for (CameraState state : {CameraState::IDLE, CameraState::CONFIGURED,
                              CameraState::RUNNING}) {
      EXPECT_TRUE(command_parser::isCommandAllowed(kind, state,
                                                   ClientRole::OBSERVER));
    }
    EXPECT_FALSE(command_parser::isCommandAllowed(kind, CameraState::IDLE,
                                                  ClientRole::COMMANDER));
    EXPECT_FALSE(command_parser::isCommandAllowed(kind, CameraState::CONFIGURED,
                                                  ClientRole::COMMANDER));
    EXPECT_TRUE(command_parser::isCommandAllowed(kind, CameraState::RUNNING,
                                                 ClientRole::COMMANDER));
  }
  // The role changes nothing else.
  EXPECT_FALSE(command_parser::isCommandAllowed(
      CommandKind::GetFrameDurationLimits, CameraState::IDLE,
      ClientRole::OBSERVER));
  EXPECT_TRUE(command_parser::isCommandAllowed(
      CommandKind::GetFrameDurationLimits, CameraState::CONFIGURED,
      ClientRole::OBSERVER));
}

// --- start_stream options ----------------------------------------------------

TEST(CommandParserTest, BuildStreamRequestDefaults) {
  auto r = command_parser::buildStreamRequest(
      nlohmann::json::parse(R"({"cmd":"start_stream","camera_id":2})"));
  ASSERT_TRUE(std::holds_alternative<command_parser::StreamRequest>(r));
  const auto& req = std::get<command_parser::StreamRequest>(r);
  EXPECT_EQ(req.camera_id, 2u);
  EXPECT_DOUBLE_EQ(req.options.max_fps, 0.0);
  EXPECT_EQ(req.options.subsample, 1u);
}

TEST(CommandParserTest, BuildStreamRequestParsesOptions) {
  auto r = command_parser::buildStreamRequest(nlohmann::json::parse(
      R"({"cmd":"start_stream","camera_id":0,"max_fps":2.5,"subsample":4})"));
  ASSERT_TRUE(std::holds_alternative<command_parser::StreamRequest>(r));
  const auto& req = std::get<command_parser::StreamRequest>(r);
  EXPECT_EQ(req.camera_id, 0u);
  EXPECT_DOUBLE_EQ(req.options.max_fps, 2.5);
  EXPECT_EQ(req.options.subsample, 4u);
}

TEST(CommandParserTest, BuildStreamRequestRejectsBadInput) {
  auto bad = [](const char* text) {
    auto r = command_parser::buildStreamRequest(nlohmann::json::parse(text));
    return std::holds_alternative<std::string>(r);
  };
  EXPECT_TRUE(bad(R"({"cmd":"start_stream"})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":"0"})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":-1})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":0,"subsample":3})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":0,"subsample":16})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":0,"subsample":0})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":0,"max_fps":-1})"));
  EXPECT_TRUE(bad(R"({"cmd":"start_stream","camera_id":0,"max_fps":"fast"})"));
  EXPECT_FALSE(bad(R"({"cmd":"start_stream","camera_id":0,"max_fps":0,"subsample":8})"));
}

TEST(CommandParserTest, StackCaptureIsCommanderOnlyAndNeedsRunning) {
  auto r = command_parser::parseCommand(R"({"cmd":"stack_capture"})");
  ASSERT_TRUE(std::holds_alternative<ParsedCommand>(r));
  EXPECT_EQ(std::get<ParsedCommand>(r).kind, CommandKind::StackCapture);
  EXPECT_TRUE(command_parser::isCommandPermitted(CommandKind::StackCapture, ClientRole::COMMANDER));
  EXPECT_FALSE(command_parser::isCommandPermitted(CommandKind::StackCapture, ClientRole::OBSERVER));
  EXPECT_TRUE(command_parser::isCommandAllowed(CommandKind::StackCapture, CameraState::RUNNING));
  EXPECT_FALSE(command_parser::isCommandAllowed(CommandKind::StackCapture, CameraState::CONFIGURED));
}

TEST(CommandParserTest, BuildStackRequestDefaults) {
  auto r = command_parser::buildStackRequest(nlohmann::json::parse(R"({"cmd":"stack_capture"})"));
  ASSERT_TRUE(std::holds_alternative<StackRequest>(r));
  const auto& req = std::get<StackRequest>(r);
  EXPECT_FALSE(req.camera_id);
  EXPECT_EQ(req.num_frames, 16u);
  EXPECT_FALSE(req.exposure_time_us);
  EXPECT_FALSE(req.analogue_gain);  // nothing pinned
  EXPECT_TRUE(req.send_image);
}

TEST(CommandParserTest, BuildStackRequestParsesFieldsAndDefaultsGainToOne) {
  auto r = command_parser::buildStackRequest(nlohmann::json::parse(
      R"({"cmd":"stack_capture","camera_id":1,"num_frames":32,
          "exposure_time":50000,"skip_frames":2,"send_image":false})"));
  ASSERT_TRUE(std::holds_alternative<StackRequest>(r));
  const auto& req = std::get<StackRequest>(r);
  EXPECT_EQ(req.camera_id, 1u);
  EXPECT_EQ(req.num_frames, 32u);
  EXPECT_EQ(req.exposure_time_us, 50000);
  ASSERT_TRUE(req.analogue_gain);
  EXPECT_FLOAT_EQ(*req.analogue_gain, 1.0f);
  EXPECT_EQ(req.skip_frames, 2);
  EXPECT_FALSE(req.send_image);

  auto g = command_parser::buildStackRequest(nlohmann::json::parse(
      R"({"exposure_time":50000,"analogue_gain":2.5})"));
  ASSERT_TRUE(std::holds_alternative<StackRequest>(g));
  EXPECT_FLOAT_EQ(*std::get<StackRequest>(g).analogue_gain, 2.5f);
}

TEST(CommandParserTest, BuildStackRequestRejectsBadInput) {
  for (const char* bad : {
           R"({"num_frames":0})", R"({"num_frames":1025})", R"({"num_frames":2.5})",
           R"({"exposure_time":0})", R"({"analogue_gain":0.5})",
           R"({"skip_frames":-1})", R"({"send_image":"yes"})",
           R"({"camera_id":-1})"}) {
    EXPECT_TRUE(std::holds_alternative<std::string>(
        command_parser::buildStackRequest(nlohmann::json::parse(bad))))
        << bad;
  }
}

TEST(CommandParserTest, RawConfigParses) {
  EXPECT_TRUE(command_parser::buildCameraConfig(nlohmann::json::parse(R"({"raw":true})")).raw);
  EXPECT_FALSE(command_parser::buildCameraConfig(nlohmann::json::parse(R"({})")).raw);
}

}  // namespace
