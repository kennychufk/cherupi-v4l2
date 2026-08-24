#include <gtest/gtest.h>

#include "capacity.hpp"

namespace {

// The geometry from the failure this module exists to prevent: an IMX708 at
// full resolution on a 1 GB Pi 5, locked to 2 fps, in a mode that retains
// every frame.
constexpr uint32_t kW = 4608;
constexpr uint32_t kH = 2592;
constexpr size_t kFrameBytes = 17915904;  // 4608 * 2592 * 3 / 2
constexpr int64_t kTwoFpsUs = 500000;
constexpr size_t kMiB = 1024ull * 1024ull;

TEST(CapacityTest, FrameBytesIsYuv420) {
  EXPECT_EQ(capacity::frameBytes(kW, kH), kFrameBytes);
  EXPECT_EQ(capacity::frameBytes(2328, 1748), 6104016u);
  EXPECT_EQ(capacity::frameBytes(0, 0), 0u);
}

TEST(CapacityTest, DefaultBudgetIsHalfOfAvailable) {
  // 800 MB available -> 400 MB budget.
  EXPECT_EQ(capacity::defaultBacklogBudgetBytes(800 * kMiB), 400 * kMiB);
}

TEST(CapacityTest, DefaultBudgetClampsToFloorAndCap) {
  // A tiny box still gets a usable floor rather than a budget of nothing.
  EXPECT_EQ(capacity::defaultBacklogBudgetBytes(16 * kMiB), 64 * kMiB);
  // A large host does not get to buffer arbitrarily.
  EXPECT_EQ(capacity::defaultBacklogBudgetBytes(64000 * kMiB), 2048 * kMiB);
  // Unreadable /proc/meminfo (0) is treated as "assume the worst".
  EXPECT_EQ(capacity::defaultBacklogBudgetBytes(0), 64 * kMiB);
}

TEST(CapacityTest, DemandUsesTheLockedFrameDuration) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  EXPECT_EQ(d.bytes_per_frame, kFrameBytes);
  EXPECT_DOUBLE_EQ(d.frames_per_second, 2.0);
  EXPECT_EQ(d.camera_count, 1u);
  EXPECT_EQ(d.bytes_per_second, kFrameBytes * 2);
}

TEST(CapacityTest, DemandFallsBackToHwMinWhenUnlocked) {
  // No FrameDurationLimits lock: the pipeline must survive the sensor's
  // fastest mode, so that is what the demand is computed against.
  const auto d = capacity::computeDemand(kW, kH, 1, 0, 69669);
  EXPECT_NEAR(d.frames_per_second, 14.35, 0.01);
  EXPECT_GT(d.bytes_per_second, kFrameBytes * 14);
}

TEST(CapacityTest, DemandScalesWithCameraCount) {
  const auto one = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  const auto four = capacity::computeDemand(kW, kH, 4, kTwoFpsUs, 69669);
  EXPECT_EQ(four.bytes_per_second, one.bytes_per_second * 4);
}

TEST(CapacityTest, DemandIsUnknownWithNoCadenceAtAll) {
  const auto d = capacity::computeDemand(kW, kH, 1, 0, 0);
  EXPECT_EQ(d.bytes_per_frame, kFrameBytes);  // geometry is still known
  EXPECT_DOUBLE_EQ(d.frames_per_second, 0.0);
  EXPECT_EQ(d.bytes_per_second, 0u);
}

// --- Disk throughput ------------------------------------------------------

TEST(CapacityTest, BatchIsInfeasibleWhenTheDiskIsTooSlow) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  // The measured rate of the SD card this was reproduced on: 21.5 MB/s
  // against a 35.8 MB/s demand.
  const auto v = capacity::checkDiskThroughput(ProcessMode::BATCH, true, d,
                                               21500000);
  EXPECT_FALSE(v.feasible);
  EXPECT_NE(v.reason.find("too slow"), std::string::npos);
  EXPECT_NE(v.reason.find("trigger"), std::string::npos);
}

TEST(CapacityTest, BatchIsFeasibleOnFastEnoughStorage) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  const auto v = capacity::checkDiskThroughput(ProcessMode::BATCH, true, d,
                                               200000000);  // 200 MB/s NVMe
  EXPECT_TRUE(v.feasible);
  EXPECT_TRUE(v.reason.empty());
}

TEST(CapacityTest, DiskCheckAbstainsForModesThatDoNotWriteEveryFrame) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  const size_t slow = 21500000;
  // TRIGGER writes only on demand; the detector modes only what they detect;
  // BUFFER is bounded by RAM, not by the disk rate.
  for (ProcessMode mode : {ProcessMode::NONE, ProcessMode::BUFFER,
                           ProcessMode::TRIGGER, ProcessMode::ARUCO2X2,
                           ProcessMode::CHECKERBOARD}) {
    EXPECT_TRUE(capacity::checkDiskThroughput(mode, true, d, slow).feasible)
        << "mode index " << static_cast<int>(mode);
  }
  // And with saving off, nothing is written at all.
  EXPECT_TRUE(
      capacity::checkDiskThroughput(ProcessMode::BATCH, false, d, slow).feasible);
}

TEST(CapacityTest, DiskCheckAbstainsWithoutAMeasurement) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  // A failed probe (0) must not be read as "infinitely slow".
  EXPECT_TRUE(capacity::checkDiskThroughput(ProcessMode::BATCH, true, d, 0)
                  .feasible);
}

// --- Buffer headroom ------------------------------------------------------

TEST(CapacityTest, BufferIsInfeasibleWhenTheBudgetHoldsUnderTwoSeconds) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  // 64 MB holds 3 frames at 17.9 MB — 1.8 s at 2 fps.
  const auto v =
      capacity::checkBufferHeadroom(ProcessMode::BUFFER, true, d, 64 * 1000000);
  EXPECT_FALSE(v.feasible);
  EXPECT_NE(v.reason.find("until stop"), std::string::npos);
  EXPECT_NE(v.reason.find("batch"), std::string::npos);
}

TEST(CapacityTest, BufferIsFeasibleWithARoomyBudget) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  // 400 MB holds 22 frames = 11 s at 2 fps: short, but a deliberate choice.
  EXPECT_TRUE(
      capacity::checkBufferHeadroom(ProcessMode::BUFFER, true, d, 400 * kMiB)
          .feasible);
}

TEST(CapacityTest, BufferCheckAbstainsForOtherModes) {
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);
  EXPECT_TRUE(
      capacity::checkBufferHeadroom(ProcessMode::BATCH, true, d, 64 * 1000000)
          .feasible);
  EXPECT_TRUE(
      capacity::checkBufferHeadroom(ProcessMode::BUFFER, false, d, 64 * 1000000)
          .feasible);
}

// --- Formatting -----------------------------------------------------------

TEST(CapacityTest, FormatBytesIsOperatorReadable) {
  EXPECT_EQ(capacity::formatBytes(0), "0 B");
  EXPECT_EQ(capacity::formatBytes(512), "512 B");
  EXPECT_EQ(capacity::formatBytes(kFrameBytes), "17.9 MB");
  EXPECT_EQ(capacity::formatBytes(kFrameBytes * 54), "967.5 MB");
  EXPECT_EQ(capacity::formatRate(21500000), "21.5 MB/s");
}

// --- The reproduced failure, as arithmetic --------------------------------

TEST(CapacityTest, TheOriginalFailingConfigIsNowRejected) {
  // 990 MiB total, ~660 MB available: the machine the hang was reproduced on.
  const size_t budget = capacity::defaultBacklogBudgetBytes(660 * 1000000);
  const auto d = capacity::computeDemand(kW, kH, 1, kTwoFpsUs, 69669);

  // The budget bounds the backlog to well under the ~967 MB that exhausted
  // this box, so the OOM can no longer happen at all.
  EXPECT_LT(budget, 400 * kMiB);
  EXPECT_LT(budget, kFrameBytes * 54);

  // 330 MB / 17.9 MB = 18 frames = 9 s at 2 fps. That is a real capture
  // window, so buffer mode is allowed to start — it now degrades to dropped
  // frames at the budget instead of killing the process.
  const auto verdict =
      capacity::checkBufferHeadroom(ProcessMode::BUFFER, true, d, budget);
  EXPECT_TRUE(verdict.feasible);
  EXPECT_GE(budget / d.bytes_per_frame, 15u);
}

}  // namespace
