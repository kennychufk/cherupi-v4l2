#pragma once

#include <atomic>
#include <string>
#include <string_view>

#include <uWS/App.h>

#include "frame_sink.hpp"

// FrameSink adapter over a uWS::WebSocket. Header-only so the template
// instantiation stays next to the rest of the uWS code and does not leak into
// the unit-test translation units.
//
// uWS is single-threaded by design — every WebSocket method must run on the
// loop thread that created the socket. The streaming thread that drives this
// sink lives outside that loop, so all sends are marshalled back via
// `loop->defer()`. Without this, the loop's cork buffer can be acquired from
// two threads at once and the server crashes with
// `Cork buffer must not be acquired without checking canCork!`.
class UwsFrameSink : public FrameSink {
 public:
  using Socket = uWS::WebSocket<false, true, int>;

  // Must be constructed on the loop thread (e.g. in the WebSocket .open
  // handler) so `uWS::Loop::get()` resolves to the right loop.
  explicit UwsFrameSink(Socket* ws)
      : ws_(ws), loop_(uWS::Loop::get()), buffered_(0) {}

  bool send(const uint8_t* data, size_t len) override {
    if (!ws_ || !loop_) return false;
    // Track buffered amount eagerly so backpressure decisions don't lag the
    // deferred-send queue; the loop callback will reconcile against the real
    // ws->getBufferedAmount() when it actually performs the send.
    buffered_.fetch_add(len, std::memory_order_relaxed);
    auto* ws = ws_;
    auto* counter = &buffered_;
    std::string payload(reinterpret_cast<const char*>(data), len);
    size_t payload_len = payload.size();
    loop_->defer(
        [ws, counter, payload = std::move(payload), payload_len]() mutable {
          ws->send(payload, uWS::OpCode::BINARY);
          counter->fetch_sub(payload_len, std::memory_order_relaxed);
        });
    return true;
  }

  size_t bufferedAmount() const override {
    // Combine the queued-but-not-yet-sent bytes (tracked here) with whatever
    // uWS reports as already-buffered. Reading ws_->getBufferedAmount() from
    // the streaming thread isn't strictly safe either, but it's a read of a
    // size_t and the value is only used for backpressure heuristics.
    size_t pending = buffered_.load(std::memory_order_relaxed);
    size_t live = ws_ ? ws_->getBufferedAmount() : 0;
    return pending + live;
  }

 private:
  Socket* ws_;
  uWS::Loop* loop_;
  std::atomic<size_t> buffered_;
};
