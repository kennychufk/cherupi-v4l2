#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "raw_stacker.hpp"

namespace {

// Records the override calls the stacker makes on the cameras.
struct Overrides {
  std::mutex m;
  std::vector<uint32_t> applied, restored;
  RawStacker::OverrideFn apply() {
    return [this](uint32_t id, std::optional<int32_t>, std::optional<float>) {
      std::lock_guard<std::mutex> lock(m);
      applied.push_back(id);
    };
  }
  RawStacker::RestoreFn restore() {
    return [this](uint32_t id) {
      std::lock_guard<std::mutex> lock(m);
      restored.push_back(id);
    };
  }
};

// A flat 8x4 raw frame at `level` DN, with the given metadata.
struct FakeCamera {
  uint32_t id = 0;
  uint32_t next_frame = 0;
  std::vector<uint16_t> buf = std::vector<uint16_t>(8 * 4, 0);

  RawFrame frame(int32_t exposure, float gain, uint16_t level = 20000) {
    std::fill(buf.begin(), buf.end(), level);
    RawFrame f;
    f.data = reinterpret_cast<const uint8_t*>(buf.data());
    f.stride = 8 * sizeof(uint16_t);
    f.width = 8;
    f.height = 4;
    f.bit_depth = 12;
    f.camera_id = id;
    f.frame_id = next_frame++;
    f.timestamp_us = 1000 * f.frame_id;
    f.exposure_time_us = exposure;
    f.analogue_gain = gain;
    f.black_level = 4096.f;
    return f;
  }
};

StackPlan plan(std::vector<uint32_t> cams, uint32_t n) {
  StackPlan p;
  p.camera_ids = std::move(cams);
  p.num_frames = n;
  p.exposure_time_us = 30000;
  p.analogue_gain = 1.0f;
  return p;
}

StackResult await(std::future<StackResult>& f) {
  EXPECT_EQ(f.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  return f.get();
}

TEST(RawStackerTest, WaitsForTheSettingsThenStacksAndRestores) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  std::promise<StackResult> done;
  auto fut = done.get_future();
  auto id = stacker.arm(plan({0}, 3), [&](StackResult r) { done.set_value(r); }, nullptr);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, 1u);
  EXPECT_EQ(ov.applied, std::vector<uint32_t>({0}));

  FakeCamera cam;
  stacker.onRawFrame(cam.frame(10000, 4.0f));  // still the preview settings
  stacker.onRawFrame(cam.frame(10000, 4.0f));
  for (int i = 0; i < 3; ++i) stacker.onRawFrame(cam.frame(29990, 1.0f));
  stacker.onRawFrame(cam.frame(29990, 1.0f));  // after the stack: ignored

  StackResult r = await(fut);
  EXPECT_EQ(r.stack_id, 1u);
  EXPECT_FALSE(r.cancelled);
  ASSERT_EQ(r.captures.size(), 1u);
  const auto& c = r.captures[0];
  EXPECT_TRUE(c.error.empty()) << c.error;
  EXPECT_EQ(c.frames_stacked, 3u);
  EXPECT_EQ(c.settle_frames, 2u);
  EXPECT_EQ(c.first_frame_id, 2u);
  EXPECT_EQ(c.last_frame_id, 4u);
  EXPECT_EQ(c.width, 4u);
  EXPECT_EQ(c.height, 2u);
  EXPECT_FLOAT_EQ(c.black_level, 4096.f);
  EXPECT_NEAR(c.white_fraction, (20000.f - 4096.f) / (65520.f - 4096.f), 1e-4);
  EXPECT_EQ(c.clipped_fraction, 0.f);
  EXPECT_EQ(ov.restored, std::vector<uint32_t>({0}));
  EXPECT_FALSE(stacker.hasPending());
}

TEST(RawStackerTest, SkipsFramesAndRestartsWhenTheSettingsChange) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  StackPlan p = plan({0}, 2);
  p.skip_frames = 2;
  std::promise<StackResult> done;
  auto fut = done.get_future();
  ASSERT_TRUE(stacker.arm(p, [&](StackResult r) { done.set_value(r); }, nullptr));

  FakeCamera cam;
  stacker.onRawFrame(cam.frame(30000, 1.0f));  // 0: settled, skipped
  stacker.onRawFrame(cam.frame(30000, 1.0f));  // 1: skipped
  stacker.onRawFrame(cam.frame(30000, 1.0f));  // 2: summed
  stacker.onRawFrame(cam.frame(30000, 1.5f));  // 3: other gain: restart at 3
  stacker.onRawFrame(cam.frame(30000, 1.0f));  // 4: differs from 3: restart
  stacker.onRawFrame(cam.frame(30000, 1.0f));  // 5: summed, done
  StackResult r = await(fut);
  ASSERT_EQ(r.captures.size(), 1u);
  EXPECT_TRUE(r.captures[0].error.empty()) << r.captures[0].error;
  EXPECT_EQ(r.captures[0].first_frame_id, 4u);
  EXPECT_EQ(r.captures[0].last_frame_id, 5u);
  EXPECT_EQ(r.captures[0].frames_rejected, 2u);
  EXPECT_EQ(r.captures[0].frames_stacked, 2u);
}

TEST(RawStackerTest, SettingsThatKeepChangingFailTheCamera) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  StackPlan p = plan({0}, 4);
  p.exposure_time_us.reset();  // auto exposure
  p.analogue_gain.reset();
  std::promise<StackResult> done;
  auto fut = done.get_future();
  ASSERT_TRUE(stacker.arm(p, [&](StackResult r) { done.set_value(r); }, nullptr));
  FakeCamera cam;
  for (int i = 0; i < 100; ++i)
    stacker.onRawFrame(cam.frame(10000 + 1000 * i, 1.0f));  // a ramping AGC
  StackResult r = await(fut);
  EXPECT_NE(r.captures[0].error.find("kept changing"), std::string::npos);
  EXPECT_TRUE(ov.applied.empty());  // nothing was pinned
}

TEST(RawStackerTest, SettingsThatNeverArriveFailTheCamera) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  std::promise<StackResult> done;
  auto fut = done.get_future();
  ASSERT_TRUE(stacker.arm(plan({0}, 2), [&](StackResult r) { done.set_value(r); }, nullptr));
  FakeCamera cam;
  for (uint32_t i = 0; i <= RawStacker::kSettleMaxFrames; ++i)
    stacker.onRawFrame(cam.frame(20000, 1.0f));  // clamped exposure
  StackResult r = await(fut);
  ASSERT_EQ(r.captures.size(), 1u);
  EXPECT_NE(r.captures[0].error.find("did not settle"), std::string::npos);
  EXPECT_EQ(r.captures[0].frames_stacked, 0u);
  EXPECT_EQ(ov.restored, std::vector<uint32_t>({0}));
}

TEST(RawStackerTest, OneStackAtATimeAndCancelRestoresAndReports) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  std::promise<StackResult> done;
  auto fut = done.get_future();
  ASSERT_TRUE(stacker.arm(plan({0, 1}, 4), [&](StackResult r) { done.set_value(r); }, nullptr));
  EXPECT_FALSE(stacker.arm(plan({0}, 1), [](StackResult) {}, nullptr).has_value());

  FakeCamera cam0;
  stacker.onRawFrame(cam0.frame(30000, 1.0f));
  stacker.cancel();
  StackResult r = await(fut);
  EXPECT_TRUE(r.cancelled);
  ASSERT_EQ(r.captures.size(), 2u);
  EXPECT_EQ(r.captures[0].frames_stacked, 1u);
  EXPECT_EQ(r.captures[0].error, "cancelled");
  EXPECT_EQ(ov.restored.size(), 2u);
  EXPECT_FALSE(stacker.hasPending());
  EXPECT_TRUE(stacker.arm(plan({0}, 1), [](StackResult) {}, nullptr).has_value());
}

TEST(RawStackerTest, WaitsForEveryArmedCamera) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  std::promise<StackResult> done;
  auto fut = done.get_future();
  ASSERT_TRUE(stacker.arm(plan({0, 1}, 1), [&](StackResult r) { done.set_value(r); }, nullptr));
  FakeCamera cam0, cam1, cam2;
  cam1.id = 1;
  cam2.id = 2;
  stacker.onRawFrame(cam0.frame(30000, 1.0f));
  stacker.onRawFrame(cam2.frame(30000, 1.0f));  // not armed
  EXPECT_EQ(fut.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  stacker.onRawFrame(cam1.frame(30000, 1.0f));
  StackResult r = await(fut);
  ASSERT_EQ(r.captures.size(), 2u);
  EXPECT_EQ(r.captures[0].camera_id, 0u);
  EXPECT_EQ(r.captures[1].camera_id, 1u);
}

TEST(RawStackerTest, CancelHandsOverNoFrames) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  std::promise<std::vector<CaptureFrame>> frames;
  auto fut = frames.get_future();
  ASSERT_TRUE(stacker.arm(plan({0}, 4), [](StackResult) {},
                          [&](std::vector<CaptureFrame> f) { frames.set_value(std::move(f)); }));
  stacker.cancel();
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_TRUE(fut.get().empty());
}

TEST(RawStackerTest, HandsTheStackedImageOverAfterTheAck) {
  Overrides ov;
  RawStacker stacker(ov.apply(), ov.restore());
  StackPlan p = plan({0, 1}, 2);
  p.send_image = false;
  p.detect.kind = ProcessMode::CHECKERBOARD;  // a flat image: nothing found
  std::mutex m;
  std::vector<std::string> order;
  std::promise<std::vector<CaptureFrame>> frames;
  auto fut = frames.get_future();
  ASSERT_TRUE(stacker.arm(
      p,
      [&](StackResult) {
        std::lock_guard<std::mutex> lock(m);
        order.push_back("ack");
      },
      [&](std::vector<CaptureFrame> f) {
        {
          std::lock_guard<std::mutex> lock(m);
          order.push_back("frames");
        }
        frames.set_value(std::move(f));
      }));
  FakeCamera cam0, cam1;
  cam1.id = 1;
  for (int i = 0; i < 4; ++i) stacker.onRawFrame(cam1.frame(30000, 1.0f));  // 2 summed
  stacker.onRawFrame(cam0.frame(30000, 1.0f, 10000));
  stacker.onRawFrame(cam0.frame(30000, 1.0f, 12000));
  ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  std::vector<CaptureFrame> out = fut.get();
  EXPECT_EQ(order, std::vector<std::string>({"ack", "frames"}));

  // One image per camera, in camera_id order.
  ASSERT_EQ(out.size(), 2u);
  const FrameData& frame = *out[0].frame;
  EXPECT_EQ(frame.camera_id, 0u);
  EXPECT_EQ(out[1].frame->camera_id, 1u);
  EXPECT_EQ(frame.frame_kind, FrameKind::STACK);
  EXPECT_EQ(frame.capture_id, 1u);  // the stack id
  EXPECT_EQ(frame.frame_id, 1u);    // the last summed frame
  EXPECT_EQ(frame.pixel_format, PIXEL_FORMAT_STACK_F32);
  EXPECT_EQ(frame.width, 4u);
  EXPECT_EQ(frame.bytes_per_line, 16u);
  ASSERT_EQ(frame.data.size(), 4u * 2u * sizeof(float));
  float v;
  std::memcpy(&v, frame.data.data(), sizeof(v));
  EXPECT_FLOAT_EQ(v, 11000.f - 4096.f);
  EXPECT_EQ(out[0].detection_kind, DetectionKind::CHECKERBOARD);
  EXPECT_TRUE(out[0].sets.empty());
  EXPECT_FALSE(out[0].send_image);
  EXPECT_FALSE(stacker.hasPending());
}

TEST(RawStackerTest, ResultJsonCarriesStatsAndErrors) {
  StackResult r;
  r.stack_id = 5;
  StackResult::Capture ok;
  ok.camera_id = 0;
  ok.frames_stacked = 8;
  StackResult::Capture bad;
  bad.camera_id = 1;
  bad.error = "boom";
  r.captures = {ok, bad};
  const nlohmann::json j = stackResultJson(r);
  EXPECT_EQ(j["type"], "stack_result");
  EXPECT_EQ(j["stack_id"], 5);
  EXPECT_EQ(j["captures"][0]["frames_stacked"], 8);
  EXPECT_FALSE(j["captures"][0].contains("error"));
  // Detection rides on the binary frame, and nothing is saved.
  EXPECT_FALSE(j["captures"][0].contains("sets"));
  EXPECT_FALSE(j["captures"][0].contains("filename"));
  EXPECT_EQ(j["captures"][1]["error"], "boom");
}

}  // namespace
