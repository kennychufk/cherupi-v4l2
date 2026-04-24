#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "rate_controller.hpp"

namespace {

TEST(AdaptiveRateControllerTest, InitialRateAndDelay) {
  AdaptiveRateController rc;
  EXPECT_DOUBLE_EQ(rc.getCurrentRate(),
                   AdaptiveRateController::INITIAL_RATE);

  // Expected delay at 200 Hz is 5 ms = 5000 us.
  auto delay = rc.getChunkDelay();
  EXPECT_NEAR(delay.count(), 5000, 100);
}

TEST(AdaptiveRateControllerTest, SuccessesBumpRateUpward) {
  AdaptiveRateController rc;
  const double initial = rc.getCurrentRate();

  // The controller increases its rate after >10 successes.
  for (int i = 0; i < 11; ++i) rc.recordChunkSent();

  EXPECT_GT(rc.getCurrentRate(), initial);
}

TEST(AdaptiveRateControllerTest, BackpressureReducesRateSignificantly) {
  AdaptiveRateController rc;
  const double initial = rc.getCurrentRate();

  rc.recordBackpressure();

  // Decrease factor 0.5 with exponential smoothing (0.7*old + 0.3*new)
  // => smoothed ≈ 0.7*200 + 0.3*100 = 170. Require at least 10% drop.
  EXPECT_LT(rc.getCurrentRate(), initial * 0.9);
}

TEST(AdaptiveRateControllerTest, ClusteredBackpressureReducesMoreAggressively) {
  AdaptiveRateController a;
  AdaptiveRateController b;

  // Single event after a long gap (>1s): uses RATE_DECREASE_FACTOR (0.5).
  // The default constructor sets last_backpressure_time 10s ago, so the first
  // event is always "not clustered".
  a.recordBackpressure();
  double a_rate = a.getCurrentRate();

  // Two events back-to-back: second one hits the clustered branch
  // (factor 0.5 * 0.7 = 0.35), giving a lower smoothed rate than `a`.
  b.recordBackpressure();
  b.recordBackpressure();
  double b_rate = b.getCurrentRate();

  EXPECT_LT(b_rate, a_rate);
}

TEST(AdaptiveRateControllerTest, RateClampsToMinimum) {
  AdaptiveRateController rc;
  // Fire many clustered backpressure events — should floor at min.
  for (int i = 0; i < 200; ++i) rc.recordBackpressure();
  EXPECT_GE(rc.getCurrentRate(),
            AdaptiveRateController::MIN_CHUNKS_PER_SECOND - 0.5);
}

TEST(AdaptiveRateControllerTest, RateClampsToMaximum) {
  AdaptiveRateController rc;
  // Fire many successes — should cap at max.
  for (int i = 0; i < 5000; ++i) rc.recordChunkSent();
  EXPECT_LE(rc.getCurrentRate(),
            AdaptiveRateController::MAX_CHUNKS_PER_SECOND + 0.5);
}

TEST(AdaptiveRateControllerTest, ResetReturnsToInitialState) {
  AdaptiveRateController rc;
  rc.recordBackpressure();
  ASSERT_NE(rc.getCurrentRate(), AdaptiveRateController::INITIAL_RATE);

  rc.reset();
  EXPECT_DOUBLE_EQ(rc.getCurrentRate(),
                   AdaptiveRateController::INITIAL_RATE);
}

}  // namespace
