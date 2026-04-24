#include "rate_controller.hpp"

#include <algorithm>

AdaptiveRateController::AdaptiveRateController() {
  last_backpressure_time =
      std::chrono::steady_clock::now() - std::chrono::seconds(10);
}

std::chrono::microseconds AdaptiveRateController::getChunkDelay() {
  std::lock_guard<std::mutex> lock(rate_mutex);
  double delay_ms = 1000.0 / smoothed_rate;
  return std::chrono::microseconds(static_cast<int64_t>(delay_ms * 1000));
}

void AdaptiveRateController::recordChunkSent() {
  std::lock_guard<std::mutex> lock(rate_mutex);

  auto now = std::chrono::steady_clock::now();
  chunk_send_times.push_back(now);
  while (chunk_send_times.size() > WINDOW_SIZE) {
    chunk_send_times.pop_front();
  }

  consecutive_successes++;
  if (consecutive_successes > 10) {
    adjustRate(RATE_INCREASE_FACTOR);
    consecutive_successes = 0;
  }
}

void AdaptiveRateController::recordBackpressure() {
  std::lock_guard<std::mutex> lock(rate_mutex);

  auto now = std::chrono::steady_clock::now();
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_backpressure_time)
                             .count();

  last_backpressure_time = now;
  backpressure_events++;
  consecutive_successes = 0;

  if (time_since_last < 1000) {
    adjustRate(RATE_DECREASE_FACTOR * 0.7);
  } else {
    adjustRate(RATE_DECREASE_FACTOR);
  }
}

void AdaptiveRateController::reset() {
  std::lock_guard<std::mutex> lock(rate_mutex);
  chunk_send_times.clear();
  current_rate = INITIAL_RATE;
  smoothed_rate = INITIAL_RATE;
  consecutive_successes = 0;
  backpressure_events = 0;
}

void AdaptiveRateController::adjustRate(double factor) {
  current_rate *= factor;
  current_rate = std::max(MIN_CHUNKS_PER_SECOND,
                          std::min(MAX_CHUNKS_PER_SECOND, current_rate));
  smoothed_rate = 0.7 * smoothed_rate + 0.3 * current_rate;
}
