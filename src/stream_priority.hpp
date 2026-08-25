#pragma once

#include <chrono>
#include <cstddef>

// Streaming priority between the two client roles.
//
// The commander's frames are mission-critical; the observer's are telemetry.
// The server cannot shape the network, so "priority" is defined as a rule on
// what the server is willing to *enqueue* for the observer: the observer's
// streamer may hand bytes to its socket only while the commander's connection
// is **clear**. The commander is clear when it is not actually consuming
// payload bandwidth (no commander, nothing streaming, or header-only), or when
// its socket shows no sign of congestion: its buffered-but-unsent bytes are
// under the soft limit, its drain handler is not reporting backpressure, and it
// has not reported backpressure recently (a hold window, so the observer does
// not burst back the instant the commander's buffer dips).
//
// Observer chunks may still interleave with commander chunks while the
// commander is uncongested — this deliberately does not serialise the two
// streams, it only stops the observer from adding to an already-stressed link.
// Pure logic so the rule is unit-testable.
namespace stream_priority {

// After a backpressure event on the higher-priority connection, how long the
// lower-priority stream keeps yielding even once the buffer has drained.
constexpr std::chrono::milliseconds kBackpressureHold{1000};

// What the lower-priority stream sees of the higher-priority connection.
struct Snapshot {
  // A higher-priority client is connected (its sink is attached).
  bool has_sink = false;
  // It is streaming at least one camera with payload (not header-only).
  bool streaming_payload = false;
  // Bytes its sink reports as buffered but not yet sent.
  size_t buffered_bytes = 0;
  // Its drain handler currently reports backpressure (hysteresis flag).
  bool in_backpressure = false;
  // Time since it last entered backpressure (a very large value if never).
  std::chrono::steady_clock::duration since_last_backpressure{
      std::chrono::hours(24)};
};

// True when a lower-priority stream may enqueue bytes right now.
inline bool isClear(const Snapshot& s, size_t soft_buffer_limit) {
  if (!s.has_sink || !s.streaming_payload) return true;
  if (s.in_backpressure) return false;
  if (s.buffered_bytes > soft_buffer_limit) return false;
  return s.since_last_backpressure >= kBackpressureHold;
}

}  // namespace stream_priority
