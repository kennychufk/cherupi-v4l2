#include <gtest/gtest.h>

#include "camera.hpp"
#include "camera_manager.hpp"
#include "types.hpp"

namespace {

TEST(CameraLifecycleTest, ConfigureAllTransitionsToConfigured) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);

  CameraConfig cfg;  // defaults
  ASSERT_TRUE(mgr.configureAll(cfg));

  for (auto* cam : mgr.getAllCameras()) {
    EXPECT_EQ(cam->getState(), CameraState::CONFIGURED);
  }
}

TEST(CameraLifecycleTest, StartStopRoundtrip) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));

  ASSERT_TRUE(mgr.startAll());
  EXPECT_TRUE(mgr.areAnyRunning());

  ASSERT_TRUE(mgr.stopAll());
  EXPECT_FALSE(mgr.areAnyRunning());
}

TEST(CameraLifecycleTest, ConfigureRejectedWhileRunning) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  ASSERT_TRUE(mgr.startAll());

  // configureAll while cameras are RUNNING returns false.
  CameraConfig cfg;
  EXPECT_FALSE(mgr.configureAll(cfg));

  // Clean up so the next test can acquire the device.
  mgr.stopAll();
}

TEST(CameraLifecycleTest, SetLensPositionAcceptedAcrossLifecycle) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  // CONFIGURED: value should be stashed; applied at start().
  mgr.setLensPosition(3.0f);
  ASSERT_TRUE(mgr.startAll());
  EXPECT_TRUE(mgr.areAnyRunning());
  // RUNNING: switch to AF, then back to a different manual value.
  mgr.setLensPosition(-1.0f);
  mgr.setLensPosition(6.0f);
  ASSERT_TRUE(mgr.stopAll());
}

TEST(CameraLifecycleTest, SetExposureTimeAcceptedAcrossLifecycle) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  // CONFIGURED: stashed, applied at start().
  mgr.setExposureTime(10000);
  ASSERT_TRUE(mgr.startAll());
  EXPECT_TRUE(mgr.areAnyRunning());
  // RUNNING: switch to auto AE, then back to a different manual value.
  mgr.setExposureTime(-1);
  mgr.setExposureTime(20000);
  ASSERT_TRUE(mgr.stopAll());
}

TEST(CameraLifecycleTest, SetFrameDurationAcceptedAcrossLifecycle) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  // CONFIGURED: stashed, applied at start().
  mgr.setFrameDuration(33333);  // ~30 fps
  EXPECT_EQ(mgr.getCurrentFrameDuration(), 33333);
  ASSERT_TRUE(mgr.startAll());
  EXPECT_TRUE(mgr.areAnyRunning());
  // RUNNING: unset, then re-lock at a different value.
  mgr.setFrameDuration(-1);
  EXPECT_EQ(mgr.getCurrentFrameDuration(), 0);
  mgr.setFrameDuration(16667);  // ~60 fps
  EXPECT_EQ(mgr.getCurrentFrameDuration(), 16667);
  ASSERT_TRUE(mgr.stopAll());
}

TEST(CameraLifecycleTest, FrameDurationLimitsHwAdvertisedAfterConfigure) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  ASSERT_TRUE(mgr.configureAll(CameraConfig{}));
  auto [lo, hi] = mgr.getFrameDurationLimitsHw();
  // Pi/IMX519 always advertises a non-trivial range. Don't pin the exact
  // numbers (tuning-dependent), just sanity-check the relationship.
  EXPECT_GT(lo, 0);
  EXPECT_GT(hi, lo);
}

}  // namespace
