#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include "rate_controller.hpp"

namespace {

TEST(AdaptiveRateControllerTest, InitialRateAndDelay) {
  AdaptiveRateController rc;
  EXPECT_DOUBLE_EQ(rc.getCurrentRate(),
                   AdaptiveRateController::INITIAL_RATE);

  // The delay is just the reciprocal of the rate, so derive the expectation
  // from the constant rather than hard-coding a figure that goes stale the
  // next time INITIAL_RATE is retuned (it already did, in 57fd041).
  const double expected_us = 1e6 / AdaptiveRateController::INITIAL_RATE;
  auto delay = rc.getChunkDelay();
  EXPECT_NEAR(delay.count(), expected_us, std::max(1.0, expected_us * 0.02));
}

TEST(AdaptiveRateControllerTest, SuccessesBumpRateUpward) {
  AdaptiveRateController rc;
  // INITIAL_RATE currently equals MAX_CHUNKS_PER_SECOND, so a fresh
  // controller has no headroom to climb into. Back off first, then verify a
  // run of successes recovers — that is the behaviour under test, and it holds
  // whatever the two constants are set to.
  rc.recordBackpressure();
  const double after_backoff = rc.getCurrentRate();
  ASSERT_LT(after_backoff, AdaptiveRateController::MAX_CHUNKS_PER_SECOND);

  // getCurrentRate() reports the *smoothed* rate, which keeps converging
  // toward the freshly-reduced target for several rounds before the 1.1x
  // increases overtake it. So assert the contract that matters — sustained
  // success recovers the rate to the ceiling — not a single step of it.
  for (int round = 0; round < 40; ++round) {
    for (int i = 0; i < 11; ++i) rc.recordChunkSent();
  }

  EXPECT_GT(rc.getCurrentRate(), after_backoff);
  EXPECT_NEAR(rc.getCurrentRate(), AdaptiveRateController::MAX_CHUNKS_PER_SECOND,
              AdaptiveRateController::MAX_CHUNKS_PER_SECOND * 0.05);
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
