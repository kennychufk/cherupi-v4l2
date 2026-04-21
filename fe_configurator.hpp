#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

extern "C" {
#include "common/pisp_common.h"
#include "frontend/pisp_fe_config.h"
}

namespace libpisp {
class FrontEnd;
}

// Wraps libpisp FrontEnd to produce a serialisable `pisp_fe_config` suitable
// for QBUF on the META_OUTPUT /dev/video7 node. Stage 3 provides a minimal
// default configuration (INPUT + STATS_CROP + AWB_STATS + RGBY enabled, full
// frame, neutral RGBY gains); Stage 5 wires the imx519 tuning values in.
class FeConfigurator {
 public:
  FeConfigurator();
  ~FeConfigurator();

  // Build + Prepare a config for the given frame geometry. Bayer order is
  // passed through to pisp_fe_global_config (default RGGB matches IMX519).
  bool init(uint32_t width, uint32_t height, uint8_t bayer_order = 0);

  const pisp_fe_config* config() const { return &prepared_; }
  size_t configSize() const { return sizeof(pisp_fe_config); }

 private:
  std::unique_ptr<libpisp::FrontEnd> fe_;
  pisp_fe_config prepared_{};
};
