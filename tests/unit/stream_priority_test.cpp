#include <gtest/gtest.h>

#include <chrono>

#include "stream_priority.hpp"

namespace {

using stream_priority::isClear;
using stream_priority::kBackpressureHold;
using stream_priority::Snapshot;

constexpr size_t kLimit = 512 * 1024;

// A commander that is streaming payload and shows no sign of congestion.
Snapshot streamingCommander() {
  Snapshot s;
  s.has_sink = true;
  s.streaming_payload = true;
  s.buffered_bytes = 0;
  s.in_backpressure = false;
  s.since_last_backpressure = std::chrono::hours(1);
  return s;
}

TEST(StreamPriorityTest, NoCommanderIsClear) {
  Snapshot s;
  s.has_sink = false;
  // Stale numbers from a previous connection must not matter.
  s.streaming_payload = true;
  s.buffered_bytes = 10 * kLimit;
  s.in_backpressure = true;
  s.since_last_backpressure = std::chrono::milliseconds(0);
  EXPECT_TRUE(isClear(s, kLimit));
}

TEST(StreamPriorityTest, CommanderNotConsumingPayloadBandwidthIsClear) {
  // Connected but not streaming (or header-only): the observer may use the
  // link however congested the commander's socket looks.
  Snapshot s = streamingCommander();
  s.streaming_payload = false;
  s.buffered_bytes = 10 * kLimit;
  s.in_backpressure = true;
  s.since_last_backpressure = std::chrono::milliseconds(0);
  EXPECT_TRUE(isClear(s, kLimit));
}

TEST(StreamPriorityTest, UncongestedStreamingCommanderIsClear) {
  EXPECT_TRUE(isClear(streamingCommander(), kLimit));
  Snapshot s = streamingCommander();
  s.buffered_bytes = kLimit;  // at the limit is still fine
  EXPECT_TRUE(isClear(s, kLimit));
}

TEST(StreamPriorityTest, BufferOverSoftLimitBlocks) {
  Snapshot s = streamingCommander();
  s.buffered_bytes = kLimit + 1;
  EXPECT_FALSE(isClear(s, kLimit));
}

TEST(StreamPriorityTest, ActiveBackpressureBlocks) {
  Snapshot s = streamingCommander();
  s.in_backpressure = true;
  EXPECT_FALSE(isClear(s, kLimit));
}

TEST(StreamPriorityTest, RecentBackpressureBlocksForTheHoldWindow) {
  Snapshot s = streamingCommander();
  s.since_last_backpressure = kBackpressureHold - std::chrono::milliseconds(1);
  EXPECT_FALSE(isClear(s, kLimit));
  s.since_last_backpressure = kBackpressureHold;
  EXPECT_TRUE(isClear(s, kLimit));
}

}  // namespace
