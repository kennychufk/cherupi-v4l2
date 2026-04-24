#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include <linux/videodev2.h>

#include "camera.hpp"
#include "camera_manager.hpp"
#include "types.hpp"

namespace {

TEST(CameraFrameCaptureTest, ProducesYuv420FramesOfExpectedSize) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  ASSERT_TRUE(mgr.startAll());

  Camera* cam = mgr.getCamera(0);
  ASSERT_NE(cam, nullptr);

  ASSERT_TRUE(cam->waitForNewFrame(std::chrono::milliseconds(2000)));

  FrameData frame;
  ASSERT_TRUE(cam->getFrameForStreaming(frame));
  cam->releaseStreamingFrame();

  EXPECT_EQ(frame.pixel_format, static_cast<uint32_t>(V4L2_PIX_FMT_YUV420));
  EXPECT_GT(frame.width, 0u);
  EXPECT_GT(frame.height, 0u);
  // YUV420 is 12 bits/pixel; frame.data holds full planar buffer.
  size_t min_expected = static_cast<size_t>(frame.width) * frame.height * 3 / 2;
  EXPECT_GE(frame.data.size(), min_expected);

  mgr.stopAll();
}

TEST(CameraFrameCaptureTest, DeliversMultipleFrames) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  ASSERT_TRUE(mgr.startAll());

  Camera* cam = mgr.getCamera(0);
  ASSERT_NE(cam, nullptr);

  int received = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received < 10 && std::chrono::steady_clock::now() < deadline) {
    if (!cam->waitForNewFrame(std::chrono::milliseconds(500))) break;
    FrameData frame;
    if (cam->getFrameForStreaming(frame)) {
      cam->releaseStreamingFrame();
      ++received;
    }
  }

  EXPECT_GE(received, 10) << "Expected at least 10 frames in 3 seconds";
  EXPECT_GE(cam->getFramesCaptured(), 10u);

  mgr.stopAll();
}

}  // namespace
