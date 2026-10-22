#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <vector>

#include "trigger_capturer.hpp"

namespace {

FrameData makeFrame(uint32_t camera_id, uint32_t frame_id) {
  FrameData f;
  f.camera_id = camera_id;
  f.frame_id = frame_id;
  f.width = 4;
  f.height = 4;
  f.bytes_per_line = 4;
  f.pixel_format = 0x32315559;  // 'YU12'
  f.data.assign(24, static_cast<uint8_t>(frame_id));
  return f;
}

TriggerPlan plan(std::vector<uint32_t> cams, int skip = 0) {
  TriggerPlan p;
  p.camera_ids = std::move(cams);
  p.skip_frames = skip;
  return p;
}

// Collects one trigger's ack and frames.
struct Outcome {
  std::promise<TriggerResult> ack;
  std::promise<std::vector<CaptureFrame>> frames;
  std::future<TriggerResult> ack_f = ack.get_future();
  std::future<std::vector<CaptureFrame>> frames_f = frames.get_future();
  TriggerCapturer::AckFn onAck() {
    return [this](TriggerResult r) { ack.set_value(std::move(r)); };
  }
  TriggerCapturer::FramesFn onFrames() {
    return [this](std::vector<CaptureFrame> f) { frames.set_value(std::move(f)); };
  }
  bool acked() {
    return ack_f.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }
  std::vector<CaptureFrame> awaitFrames() {
    EXPECT_EQ(frames_f.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    return frames_f.get();
  }
};

TEST(TriggerCapturerTest, IgnoresFramesUntilArmed) {
  TriggerCapturer capturer(nullptr);
  capturer.onFrame(makeFrame(0, 0));
  EXPECT_FALSE(capturer.hasPending());
}

TEST(TriggerCapturerTest, CapturesTheNextFrameOfEveryArmedCamera) {
  TriggerCapturer capturer(nullptr);
  Outcome out;
  auto id = capturer.arm(plan({0, 1}), out.onAck(), out.onFrames());
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, 1u);
  EXPECT_TRUE(capturer.hasPending());

  // Camera 1 delivers first: the trigger keeps waiting on camera 0, and a
  // second frame from camera 1 is not taken again.
  capturer.onFrame(makeFrame(1, 10));
  capturer.onFrame(makeFrame(1, 11));
  capturer.onFrame(makeFrame(2, 5));  // not armed
  EXPECT_FALSE(out.acked());
  capturer.onFrame(makeFrame(0, 20));

  // The ack comes from the last camera's call, before any detection.
  ASSERT_TRUE(out.acked());
  TriggerResult r = out.ack_f.get();
  EXPECT_FALSE(capturer.hasPending());
  EXPECT_EQ(r.trigger_id, 1u);
  EXPECT_FALSE(r.cancelled);
  ASSERT_EQ(r.captures.size(), 2u);  // delivery order
  EXPECT_EQ(r.captures[0].camera_id, 1u);
  EXPECT_EQ(r.captures[0].frame_id, 10u);
  EXPECT_EQ(r.captures[1].camera_id, 0u);
  EXPECT_EQ(r.captures[1].frame_id, 20u);

  std::vector<CaptureFrame> frames = out.awaitFrames();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].frame->camera_id, 1u);
  EXPECT_EQ(frames[0].frame->frame_id, 10u);
  EXPECT_EQ(frames[0].frame->frame_kind, FrameKind::TRIGGER);
  EXPECT_EQ(frames[0].frame->capture_id, 1u);
  EXPECT_EQ(frames[0].frame->data, makeFrame(1, 10).data);
  EXPECT_EQ(frames[0].detection_kind, DetectionKind::NONE);
  EXPECT_TRUE(frames[0].send_image);
  EXPECT_EQ(frames[1].frame->frame_id, 20u);
}

TEST(TriggerCapturerTest, SkipFramesDiscardsSettlingFrames) {
  TriggerCapturer capturer(nullptr);
  Outcome out;
  ASSERT_TRUE(capturer.arm(plan({0}, 2), out.onAck(), out.onFrames()));
  capturer.onFrame(makeFrame(0, 10));  // skipped
  capturer.onFrame(makeFrame(0, 11));  // skipped
  EXPECT_FALSE(out.acked());
  capturer.onFrame(makeFrame(0, 12));
  ASSERT_TRUE(out.acked());
  EXPECT_EQ(out.ack_f.get().captures.at(0).frame_id, 12u);
  EXPECT_EQ(out.awaitFrames().at(0).frame->frame_id, 12u);
}

TEST(TriggerCapturerTest, RunsTheDetectorAndCarriesSendImage) {
  std::vector<uint32_t> detected;
  std::mutex m;
  TriggerCapturer capturer([&](const FrameData& f) {
    std::lock_guard<std::mutex> lock(m);
    detected.push_back(f.frame_id);
    return std::vector<CornerSet>{{0, 7, {{1.f, 2.f}}}};
  });
  TriggerPlan p = plan({0});
  p.send_image = false;
  p.detection_kind = DetectionKind::ARUCO;
  Outcome out;
  ASSERT_TRUE(capturer.arm(p, out.onAck(), out.onFrames()));
  capturer.onFrame(makeFrame(0, 3));
  std::vector<CaptureFrame> frames = out.awaitFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(detected, std::vector<uint32_t>({3}));
  EXPECT_EQ(frames[0].detection_kind, DetectionKind::ARUCO);
  ASSERT_EQ(frames[0].sets.size(), 1u);
  EXPECT_EQ(frames[0].sets[0].marker_id, 7);
  EXPECT_FALSE(frames[0].send_image);
}

TEST(TriggerCapturerTest, NoDetectorRunsInAModeWithoutOne) {
  int calls = 0;
  TriggerCapturer capturer([&](const FrameData&) {
    ++calls;
    return std::vector<CornerSet>{};
  });
  Outcome out;
  ASSERT_TRUE(capturer.arm(plan({0}), out.onAck(), out.onFrames()));
  capturer.onFrame(makeFrame(0, 1));
  out.awaitFrames();
  EXPECT_EQ(calls, 0);
}

TEST(TriggerCapturerTest, OneTriggerAtATimeThenIdsCount) {
  TriggerCapturer capturer(nullptr);
  Outcome first;
  ASSERT_TRUE(capturer.arm(plan({0}), first.onAck(), first.onFrames()));
  EXPECT_FALSE(capturer.arm(plan({0}), nullptr, nullptr).has_value());
  EXPECT_FALSE(capturer.arm(plan({}), nullptr, nullptr).has_value());
  capturer.onFrame(makeFrame(0, 0));
  first.awaitFrames();

  Outcome second;
  auto id = capturer.arm(plan({0}), second.onAck(), second.onFrames());
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, 2u);
  capturer.onFrame(makeFrame(0, 1));
  second.awaitFrames();

  // start_cameras: ids start over.
  capturer.reset();
  Outcome third;
  id = capturer.arm(plan({0}), third.onAck(), third.onFrames());
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, 1u);
  capturer.cancel();
  third.awaitFrames();
}

TEST(TriggerCapturerTest, CancelAcksAndDeliversWhatWasTaken) {
  TriggerCapturer capturer(nullptr);
  Outcome out;
  ASSERT_TRUE(capturer.arm(plan({0, 1}), out.onAck(), out.onFrames()));
  capturer.onFrame(makeFrame(0, 7));  // camera 1 never delivers
  capturer.cancel();

  ASSERT_TRUE(out.acked());
  TriggerResult r = out.ack_f.get();
  EXPECT_TRUE(r.cancelled);
  ASSERT_EQ(r.captures.size(), 1u);
  EXPECT_EQ(r.captures[0].camera_id, 0u);
  std::vector<CaptureFrame> frames = out.awaitFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].frame->frame_id, 7u);
  EXPECT_FALSE(capturer.hasPending());

  // Cancelling with nothing armed does nothing.
  capturer.cancel();
}

TEST(TriggerCapturerTest, ResultJson) {
  TriggerResult r;
  r.trigger_id = 4;
  r.captures = {{1, 12}};
  const nlohmann::json j = triggerResultJson(r);
  EXPECT_EQ(j["type"], "trigger_result");
  EXPECT_EQ(j["trigger_id"], 4);
  EXPECT_EQ(j["cancelled"], false);
  EXPECT_EQ(j["captures"][0]["camera_id"], 1);
  EXPECT_EQ(j["captures"][0]["frame_id"], 12);
  EXPECT_FALSE(j["captures"][0].contains("filename"));
}

}  // namespace
