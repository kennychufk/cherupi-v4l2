#include "fe_configurator.hpp"

#include <cstring>
#include <iostream>

#include "frontend/frontend.hpp"
#include "variants/variant.hpp"

FeConfigurator::FeConfigurator() = default;
FeConfigurator::~FeConfigurator() = default;

bool FeConfigurator::init(uint32_t width, uint32_t height,
                          uint8_t bayer_order) {
  // BCM2712 C0 is the Pi 5 PiSP. Use the exported variant directly rather
  // than version lookup, which requires matching versions we don't know.
  fe_ = std::make_unique<libpisp::FrontEnd>(/*streaming=*/false,
                                            libpisp::BCM2712_C0);

  pisp_fe_global_config global{};
  global.enables = PISP_FE_ENABLE_INPUT | PISP_FE_ENABLE_STATS_CROP |
                   PISP_FE_ENABLE_BLA | PISP_FE_ENABLE_BLC |
                   PISP_FE_ENABLE_AWB_STATS | PISP_FE_ENABLE_AGC_STATS;
  global.bayer_order = bayer_order;
  fe_->SetGlobal(global);

  // IMX519 black level = 4096 (16-bit scale). BLA normalises per-channel
  // black levels to a common pedestal; BLC then removes that pedestal so the
  // stats hardware sees zero-based pixel values.
  constexpr uint16_t kBlackLevel = 4096;
  pisp_bla_config bla{};
  bla.black_level_r = bla.black_level_gr = bla.black_level_gb =
      bla.black_level_b = kBlackLevel;
  bla.output_black_level = kBlackLevel;
  fe_->SetBla(bla);

  pisp_bla_config blc{};
  blc.black_level_r = blc.black_level_gr = blc.black_level_gb =
      blc.black_level_b = kBlackLevel;
  blc.output_black_level = 0;
  fe_->SetBlc(blc);

  pisp_fe_input_config input{};
  input.streaming = 1;
  input.format.width = static_cast<uint16_t>(width);
  input.format.height = static_cast<uint16_t>(height);
  input.format.format = PISP_IMAGE_FORMAT_BPS_16;
  input.format.stride = static_cast<int32_t>(width * 2);
  fe_->SetInput(input);

  pisp_fe_crop_config stats_crop{};
  stats_crop.offset_x = 0;
  stats_crop.offset_y = 0;
  stats_crop.width = static_cast<uint16_t>(width);
  stats_crop.height = static_cast<uint16_t>(height);
  fe_->SetStatsCrop(stats_crop);

  // Follow libcamera pisp.cpp::setStatsAndDebin: leave offset/size at zero
  // so the Frontend lays down its native PISP_AWB_STATS_SIZE x N grid over
  // the stats_crop region. Only set the thresholds.
  pisp_fe_awb_stats_config awb{};
  awb.r_lo = awb.g_lo = awb.b_lo = 0;
  awb.r_hi = awb.g_hi = awb.b_hi = static_cast<uint16_t>(65535 * 0.98);
  fe_->SetAwbStats(awb);

  // AGC stats with auto-computed zone sizes (size_x/y = 0 → libpisp fills in
  // the correct 16×16 grid dimensions). Weights stay zero — only floating[0]
  // is used (full-image luminance for lux estimation).
  pisp_fe_agc_stats_config agc{};
  fe_->SetAgcStats(agc);

  // Floating region 0 covers the whole image — the lux algorithm reads
  // agc.floating[0].Y_sum / counted to get mean scene brightness.
  pisp_fe_floating_stats_config floating{};
  floating.regions[0].size_x = static_cast<uint16_t>(width);
  floating.regions[0].size_y = static_cast<uint16_t>(height);
  fe_->SetFloatingStats(floating);

  fe_->Prepare(&prepared_);
  return true;
}
