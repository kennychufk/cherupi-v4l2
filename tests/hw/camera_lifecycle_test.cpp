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

}  // namespace
