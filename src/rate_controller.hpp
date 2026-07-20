#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>

// Adaptive rate controller for paced chunk sending.
// - Tracks recent send timestamps in a bounded window.
// - Increases the smoothed send rate after a run of successes.
// - Decreases it on backpressure, more aggressively if events are clustered.
// Pure logic: no I/O, no transport coupling, safe to unit-test.
class AdaptiveRateController {
 public:
  static constexpr size_t WINDOW_SIZE = 20;
  static constexpr double MIN_CHUNKS_PER_SECOND = 50.0;
  static constexpr double MAX_CHUNKS_PER_SECOND = 4000.0;
  static constexpr double RATE_INCREASE_FACTOR = 1.1;
  static constexpr double RATE_DECREASE_FACTOR = 0.5;
  static constexpr double INITIAL_RATE = 4000.0;

  AdaptiveRateController();

  std::chrono::microseconds getChunkDelay();
  void recordChunkSent();
  void recordBackpressure();
  void reset();

  double getCurrentRate() const { return smoothed_rate; }

 private:
  void adjustRate(double factor);

  std::deque<std::chrono::steady_clock::time_point> chunk_send_times;
  std::chrono::steady_clock::time_point last_backpressure_time;
  double current_rate{INITIAL_RATE};
  double smoothed_rate{INITIAL_RATE};
  std::atomic<int> consecutive_successes{0};
  std::atomic<int> backpressure_events{0};
  mutable std::mutex rate_mutex;
};
