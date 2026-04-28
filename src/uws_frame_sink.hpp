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
//
// Lifetime: deferred lambdas can outlive the close handler — uWS frees the
// underlying `Socket*` once the close handler returns, but lambdas already
// queued before the close still run afterwards. To stay safe we
//   * own the sink via `shared_ptr` so each deferred lambda extends its
//     lifetime (keeps `buffered_` alive past the close handler), and
//   * gate `ws_->send` on a `valid_` flag that the close handler clears
//     before releasing its `shared_ptr`, so we never touch the freed socket.
class UwsFrameSink : public FrameSink,
                     public std::enable_shared_from_this<UwsFrameSink> {
 public:
  using Socket = uWS::WebSocket<false, true, int>;

  // Must be constructed on the loop thread (e.g. in the WebSocket .open
  // handler) so `uWS::Loop::get()` resolves to the right loop.
  explicit UwsFrameSink(Socket* ws)
      : ws_(ws), loop_(uWS::Loop::get()), buffered_(0), valid_(true) {}

  bool send(const uint8_t* data, size_t len) override {
    if (!ws_ || !loop_) return false;
    if (!valid_.load(std::memory_order_acquire)) return false;
    // Track buffered amount eagerly so backpressure decisions don't lag the
    // deferred-send queue; the loop callback will reconcile against the real
    // ws->getBufferedAmount() when it actually performs the send.
    buffered_.fetch_add(len, std::memory_order_relaxed);
    std::string payload(reinterpret_cast<const char*>(data), len);
    size_t payload_len = payload.size();
    auto self = shared_from_this();
    loop_->defer(
        [self = std::move(self), payload = std::move(payload),
         payload_len]() mutable {
          if (self->valid_.load(std::memory_order_acquire)) {
            self->ws_->send(payload, uWS::OpCode::BINARY);
          }
          self->buffered_.fetch_sub(payload_len, std::memory_order_relaxed);
        });
    return true;
  }

  size_t bufferedAmount() const override {
    // Combine the queued-but-not-yet-sent bytes (tracked here) with whatever
    // uWS reports as already-buffered. Reading ws_->getBufferedAmount() from
    // the streaming thread isn't strictly safe either, but it's a read of a
    // size_t and the value is only used for backpressure heuristics.
    size_t pending = buffered_.load(std::memory_order_relaxed);
    size_t live = (ws_ && valid_.load(std::memory_order_acquire))
                      ? ws_->getBufferedAmount()
                      : 0;
    return pending + live;
  }

  // Mark the sink unusable. After this call, send() refuses new work and any
  // already-deferred lambdas will skip the ws_->send call when they run.
  // Must be called on the loop thread before the underlying Socket* is freed.
  void invalidate() { valid_.store(false, std::memory_order_release); }

 private:
  Socket* ws_;
  uWS::Loop* loop_;
  std::atomic<size_t> buffered_;
  std::atomic<bool> valid_;
};
