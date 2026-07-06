#include <gtest/gtest.h>

#include "camera_manager.hpp"

namespace {

TEST(CameraDiscoveryTest, FindsAtLeastOneImx519) {
  CameraManager mgr;
  size_t count = mgr.discoverCameras();
  ASSERT_GE(count, 1u) << "Expected at least one IMX519 attached";
  EXPECT_EQ(count, mgr.getCameraCount());
  EXPECT_NE(mgr.getCamera(0)->getModel().find("imx519"), std::string::npos);
}

TEST(CameraDiscoveryTest, GetCameraBoundsCheck) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);

  EXPECT_NE(mgr.getCamera(0), nullptr);
  EXPECT_EQ(mgr.getCamera(99), nullptr);
}

TEST(CameraDiscoveryTest, DiscoverIsIdempotent) {
  CameraManager mgr;
  size_t first = mgr.discoverCameras();
  ASSERT_GE(first, 1u);
  size_t second = mgr.discoverCameras();
  EXPECT_EQ(first, second);
}

TEST(CameraDiscoveryTest, GetAllCamerasMatchesCount) {
  CameraManager mgr;
  ASSERT_GE(mgr.discoverCameras(), 1u);
  EXPECT_EQ(mgr.getAllCameras().size(), mgr.getCameraCount());
}

}  // namespace
