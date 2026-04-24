#pragma once

#include <cstddef>
#include <cstdint>

// Abstract byte sink used by StreamManager.
// Production implementation wraps a uWS::WebSocket (see uws_frame_sink.hpp);
// tests inject a recording implementation to assert on the emitted byte stream
// without involving a real socket.
struct FrameSink {
  virtual ~FrameSink() = default;

  // Send a binary message. Returns true on success, false if the sink is
  // disconnected or the underlying transport threw.
  virtual bool send(const uint8_t* data, size_t len) = 0;

  // Bytes currently buffered in the transport (for backpressure diagnostics).
  virtual size_t bufferedAmount() const = 0;
};
