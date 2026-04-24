#pragma once

#include <string_view>

#include <uWS/App.h>

#include "frame_sink.hpp"

// FrameSink adapter over a uWS::WebSocket. Header-only so the template
// instantiation stays next to the rest of the uWS code and does not leak into
// the unit-test translation units.
class UwsFrameSink : public FrameSink {
 public:
  using Socket = uWS::WebSocket<false, true, int>;

  explicit UwsFrameSink(Socket* ws) : ws_(ws) {}

  bool send(const uint8_t* data, size_t len) override {
    if (!ws_) return false;
    try {
      std::string_view view(reinterpret_cast<const char*>(data), len);
      ws_->send(view, uWS::OpCode::BINARY);
      return true;
    } catch (...) {
      return false;
    }
  }

  size_t bufferedAmount() const override {
    return ws_ ? ws_->getBufferedAmount() : 0;
  }

 private:
  Socket* ws_;
};
