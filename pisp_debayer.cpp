// Standalone debayerer for RGGB_PISP_COMP1 raw frames saved by FrameSaver.
//
// File conventions (from frame_saver.cpp):
//   Raw frames : <dir>/cam<ID>-<frame_id>.raw
//   AWB sidecar: <dir>/cam<ID>_awb.bin  (packed AwbRecord[], 20 bytes each)
//
// Usage:
//   pisp_debayer [options] <input.raw> <output.[png|jpg|ppm|tiff]>
//   pisp_debayer [options] --batch <input_dir> <output_dir>
//
// Options:
//   --width   W   Sensor width in pixels  (default: 2328)
//   --height  H   Sensor height in pixels (default: 1748)
//   --stride  S   Bytes per row           (default: 2368)
//   --awb-r   R   Red   AWB gain          (default: from sidecar or 1.0)
//   --awb-b   B   Blue  AWB gain          (default: from sidecar or 1.0)
//   --quality Q   Debayer algorithm: bilinear(default), ea, vng
//   --tuning  F   libcamera tuning JSON (enables CCM + sensor gamma curve)
//   --gamma   G   Fallback power-law gamma (default: 2.2; 0 disables; ignored when --tuning is used)
//   --rotate180   Rotate output 180 degrees (matches web client orientation)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ─── PISP_COMP1 decompression ────────────────────────────────────────────────
// Ported verbatim from rpicam-apps/image/dng.cpp.
// COMPRESS_MODE=1 means the "subblock" path is used.
// COMPRESS_OFFSET=2048 is added by postprocess().

static constexpr int COMPRESS_OFFSET = 2048;

static uint16_t pisp_dequantize(uint16_t q, int qmode) {
  switch (qmode) {
    case 0: return (q < 320) ? 16 * q : 32 * (q - 160);
    case 1: return 64 * q;
    case 2: return 128 * q;
    default:
      return (q < 94) ? 256 * q
                      : static_cast<uint16_t>(std::min(0xFFFF, 512 * (q - 47)));
  }
}

static uint16_t pisp_postprocess(uint16_t a) {
  return static_cast<uint16_t>(std::min(0xFFFF, (int)a + COMPRESS_OFFSET));
}

// Fills d[0], d[2], d[4], d[6] from one 32-bit word (4 pixels, stride 2).
static void pisp_subblock(uint16_t* d, uint32_t w) {
  int q[4];
  int qmode = w & 3;
  if (qmode < 3) {
    int field0 = (w >> 2) & 511;
    int field1 = (w >> 11) & 127;
    int field2 = (w >> 18) & 127;
    int field3 = (w >> 25) & 127;
    if (qmode == 2 && field0 >= 384) {
      q[1] = field0;
      q[2] = field1 + 384;
    } else {
      q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
      q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
    }
    int p1 = std::max(0, q[1] - 64);
    if (qmode == 2) p1 = std::min(384, p1);
    int p2 = std::max(0, q[2] - 64);
    if (qmode == 2) p2 = std::min(384, p2);
    q[0] = p1 + field2;
    q[3] = p2 + field3;
  } else {
    int pack0 = (w >> 2) & 32767;
    int pack1 = (w >> 17) & 32767;
    q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
    q[1] = (pack0 >> 4) % 176;
    q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
    q[3] = (pack1 >> 4) % 176;
  }
  d[0] = pisp_dequantize(static_cast<uint16_t>(q[0]), qmode);
  d[2] = pisp_dequantize(static_cast<uint16_t>(q[1]), qmode);
  d[4] = pisp_dequantize(static_cast<uint16_t>(q[2]), qmode);
  d[6] = pisp_dequantize(static_cast<uint16_t>(q[3]), qmode);
}

// Decompress one row of PISP_COMP1 encoded data to 16-bit linear Bayer.
// src  : pointer to the start of the compressed row (stride bytes)
// dest : output buffer, width uint16_t values
// width: must be a multiple of 8
static void pisp_uncompress_row(const uint8_t* src, uint16_t* dest, int width) {
  for (int x = 0; x < width; x += 8) {
    uint32_t w0 = 0, w1 = 0;
    for (int b = 0; b < 4; ++b) w0 |= static_cast<uint32_t>(*src++) << (b * 8);
    for (int b = 0; b < 4; ++b) w1 |= static_cast<uint32_t>(*src++) << (b * 8);
    pisp_subblock(dest + x,     w0);   // even pixels: x+0, x+2, x+4, x+6
    pisp_subblock(dest + x + 1, w1);   // odd  pixels: x+1, x+3, x+5, x+7
    for (int i = 0; i < 8; ++i)
      dest[x + i] = pisp_postprocess(dest[x + i]);
  }
}

// Decompress full PISP_COMP1 frame.
// Returns a flat uint16_t array, row-major, no padding (width × height values).
// Values are in the range [COMPRESS_OFFSET, ~COMPRESS_OFFSET + sensor_max].
static std::vector<uint16_t> pisp_decompress(const uint8_t* src, int width,
                                              int height, int stride) {
  // Output width rounded up to 8 (the subblock granularity)
  int aligned_width = (width + 7) & ~7;
  std::vector<uint16_t> out(static_cast<size_t>(aligned_width) * height);

  for (int y = 0; y < height; ++y) {
    pisp_uncompress_row(src + y * stride,
                        out.data() + y * aligned_width, aligned_width);
  }
  return out;  // caller uses actual `width` when building the Mat
}

// ─── AWB sidecar ─────────────────────────────────────────────────────────────

// Must match frame_saver.hpp AwbRecord (packed, 20 bytes)
struct AwbRecord {
  uint32_t frame_id;
  uint32_t camera_id;
  float gain_r;
  float gain_b;
  float cct;
} __attribute__((packed));
static_assert(sizeof(AwbRecord) == 20, "AwbRecord size mismatch");

static std::map<uint32_t, AwbRecord> loadAwbSidecar(const fs::path& path) {
  std::map<uint32_t, AwbRecord> m;
  std::ifstream f(path, std::ios::binary);
  AwbRecord rec;
  while (f.read(reinterpret_cast<char*>(&rec), sizeof(rec)))
    m[rec.frame_id] = rec;
  return m;
}

// ─── Tuning data (from libcamera JSON) ───────────────────────────────────────

struct CcmEntry {
  float ct;
  float m[9];  // row-major RGB→RGB matrix from rpi.ccm
};

struct TuningData {
  std::vector<CcmEntry> ccms;  // sorted ascending by ct
  std::vector<float> gamma_x;  // piecewise-linear breakpoints, 16-bit input
  std::vector<float> gamma_y;  // piecewise-linear breakpoints, 16-bit output
  bool has_ccm   = false;
  bool has_gamma = false;
};

static TuningData loadTuningData(const fs::path& path) {
  TuningData td;
  std::ifstream f(path);
  if (!f) { std::cerr << "Cannot open tuning file: " << path << "\n"; return td; }

  nlohmann::json root;
  try { f >> root; }
  catch (const std::exception& e) {
    std::cerr << "Failed to parse tuning file: " << e.what() << "\n"; return td;
  }

  for (const auto& algo : root.at("algorithms")) {
    if (algo.contains("rpi.ccm")) {
      for (const auto& entry : algo["rpi.ccm"]["ccms"]) {
        CcmEntry ce;
        ce.ct = entry["ct"].get<float>();
        const auto& mv = entry["ccm"];
        for (int i = 0; i < 9; ++i) ce.m[i] = mv[i].get<float>();
        td.ccms.push_back(ce);
      }
      std::sort(td.ccms.begin(), td.ccms.end(),
                [](const CcmEntry& a, const CcmEntry& b) { return a.ct < b.ct; });
      td.has_ccm = !td.ccms.empty();
    }
    if (algo.contains("rpi.contrast")) {
      const auto& gc = algo["rpi.contrast"]["gamma_curve"];
      for (size_t i = 0; i + 1 < gc.size(); i += 2) {
        td.gamma_x.push_back(gc[i].get<float>());
        td.gamma_y.push_back(gc[i + 1].get<float>());
      }
      td.has_gamma = !td.gamma_x.empty();
    }
  }
  return td;
}

// Build a 256-entry gamma LUT from the tuning piecewise-linear curve.
// The curve is in 16-bit (0–65535) space; we map 8-bit in → 8-bit out.
static void buildTuningGammaLut(const TuningData& td, uint8_t lut[256]) {
  const auto& gx = td.gamma_x;
  const auto& gy = td.gamma_y;
  for (int i = 0; i < 256; ++i) {
    float x16 = i * (65535.0f / 255.0f);
    float y16;
    if (x16 <= gx.front()) {
      y16 = gy.front();
    } else if (x16 >= gx.back()) {
      y16 = gy.back();
    } else {
      auto it = std::lower_bound(gx.begin(), gx.end(), x16);
      int k = static_cast<int>(it - gx.begin()) - 1;
      float t = (x16 - gx[k]) / (gx[k + 1] - gx[k]);
      y16 = gy[k] + t * (gy[k + 1] - gy[k]);
    }
    lut[i] = static_cast<uint8_t>(
        std::round(std::clamp(y16 * (255.0f / 65535.0f), 0.0f, 255.0f)));
  }
}

// Interpolate (or clamp) the CCM at the given colour temperature.
// Returns the 9 coefficients in RGB row-major order.
static void interpolateCcm(const TuningData& td, float cct, float out[9]) {
  const auto& c = td.ccms;
  if (cct <= c.front().ct) { std::copy(c.front().m, c.front().m + 9, out); return; }
  if (cct >= c.back().ct)  { std::copy(c.back().m,  c.back().m  + 9, out); return; }
  int hi = 0;
  while (c[hi].ct < cct) ++hi;
  int lo = hi - 1;
  float t = (cct - c[lo].ct) / (c[hi].ct - c[lo].ct);
  for (int i = 0; i < 9; ++i) out[i] = c[lo].m[i] * (1.0f - t) + c[hi].m[i] * t;
}

// Build a 3×3 CV_32F matrix for cv::transform from an RGB-order CCM.
// cv::transform operates on BGR column vectors, so rows/columns are reordered.
static cv::Mat ccmToBgrMat(const float m[9]) {
  // JSON layout (RGB→RGB, row-major):
  //   new_R = m[0]*R + m[1]*G + m[2]*B
  //   new_G = m[3]*R + m[4]*G + m[5]*B
  //   new_B = m[6]*R + m[7]*G + m[8]*B
  //
  // cv::transform on BGR input [B,G,R]^T:
  //   new_B row → [m[8], m[7], m[6]]
  //   new_G row → [m[5], m[4], m[3]]
  //   new_R row → [m[2], m[1], m[0]]
  float data[9] = {
    m[8], m[7], m[6],
    m[5], m[4], m[3],
    m[2], m[1], m[0],
  };
  return cv::Mat(3, 3, CV_32F, data).clone();
}

// ─── Config ──────────────────────────────────────────────────────────────────

struct Config {
  bool batch = false;
  int width = 2328;
  int height = 1748;
  int stride = 2368;
  float gain_r = 1.0f;
  float gain_b = 1.0f;
  bool gains_from_cli = false;
  std::string quality = "bilinear";
  bool rotate180 = false;
  // IMX519 black level = 64 (10-bit) × 64 (scale to 16-bit) = 4096
  // White level = 65535 (full 16-bit range)
  int black_level = 4096;
  int white_level = 65535;
  bool auto_stretch = false;  // per-frame min/max normalisation
  float gamma = 2.2f;         // fallback power-law gamma; ignored when tuning loaded
  std::string tuning_path;    // path to libcamera tuning JSON
};

// ─── Filename helpers ─────────────────────────────────────────────────────────

// Parse "cam<ID>-<frame_id>" stem → (cam_id, frame_id)
static std::optional<std::pair<uint32_t, uint32_t>> parseRawStem(
    const std::string& stem) {
  if (stem.substr(0, 3) != "cam") return std::nullopt;
  auto dash = stem.find('-', 3);
  if (dash == std::string::npos) return std::nullopt;
  try {
    return std::make_pair(
        static_cast<uint32_t>(std::stoul(stem.substr(3, dash - 3))),
        static_cast<uint32_t>(std::stoul(stem.substr(dash + 1))));
  } catch (...) {
    return std::nullopt;
  }
}

// ─── Core debayer ─────────────────────────────────────────────────────────────

static bool debayerFile(const fs::path& input, const fs::path& output,
                        const Config& cfg,
                        const std::map<uint32_t, AwbRecord>* awb_map,
                        std::optional<uint32_t> frame_id,
                        const TuningData* tuning) {
  size_t expected = static_cast<size_t>(cfg.stride) * cfg.height;

  std::ifstream f(input, std::ios::binary);
  if (!f) { std::cerr << "Cannot open: " << input << "\n"; return false; }
  std::vector<uint8_t> raw(expected);
  if (!f.read(reinterpret_cast<char*>(raw.data()), expected)) {
    std::cerr << "Short read: " << input << " (expected " << expected << ")\n";
    return false;
  }

  // Resolve AWB gains and CCT
  float gain_r = cfg.gain_r, gain_b = cfg.gain_b, cct = 0.0f;
  if (!cfg.gains_from_cli && awb_map && frame_id) {
    auto it = awb_map->find(*frame_id);
    if (it != awb_map->end()) {
      gain_r = it->second.gain_r;
      gain_b = it->second.gain_b;
      cct    = it->second.cct;
    }
  }

  // ── Decompress PISP_COMP1 → 16-bit Bayer ──────────────────────────────────
  int aligned_width = (cfg.width + 7) & ~7;
  std::vector<uint16_t> bayer16 =
      pisp_decompress(raw.data(), cfg.width, cfg.height, cfg.stride);

  // ── Convert to 8-bit Bayer for OpenCV debayer ─────────────────────────────
  // Decompressed values are in 16-bit sensor space (0–65535).
  // Black level = 4096 (IMX519 pedestal 64 in 10-bit, scaled ×64 to 16-bit).
  // White level = 65535.
  //
  // If auto-stretch is enabled, derive black/white from this frame's data;
  // otherwise use the fixed sensor constants.
  int blk = cfg.black_level;
  int wht = cfg.white_level;
  if (cfg.auto_stretch) {
    uint16_t lo = 0xFFFF, hi = 0;
    for (int y = 0; y < cfg.height; ++y) {
      const uint16_t* row = bayer16.data() + y * aligned_width;
      for (int x = 0; x < cfg.width; ++x) {
        if (row[x] < lo) lo = row[x];
        if (row[x] > hi) hi = row[x];
      }
    }
    blk = lo; wht = hi;
  }
  int range = std::max(1, wht - blk);

  std::vector<uint8_t> bayer8(static_cast<size_t>(cfg.width) * cfg.height);
  for (int y = 0; y < cfg.height; ++y) {
    const uint16_t* src = bayer16.data() + y * aligned_width;
    uint8_t* dst = bayer8.data() + y * cfg.width;
    for (int x = 0; x < cfg.width; ++x) {
      int v = static_cast<int>(src[x]) - blk;
      dst[x] = static_cast<uint8_t>(std::clamp(v * 255 / range, 0, 255));
    }
  }

  // ── OpenCV debayer (BGGR = COLOR_BayerBG) ─────────────────────────────────
  cv::Mat bayer_mat(cfg.height, cfg.width, CV_8UC1, bayer8.data());

  int code;
  if      (cfg.quality == "ea")  code = cv::COLOR_BayerBG2BGR_EA;
  else if (cfg.quality == "vng") code = cv::COLOR_BayerBG2BGR_VNG;
  else                           code = cv::COLOR_BayerBG2BGR;

  cv::Mat bgr;
  cv::cvtColor(bayer_mat, bgr, code);

  // ── Float pipeline: AWB gains → CCM → clamp → gamma ──────────────────────
  // Work in float (0–255 range) so CCM cross-channel mixing doesn't clip.
  cv::Mat flt;
  bgr.convertTo(flt, CV_32FC3);

  // AWB gains (BGR order: B=ch0, G=ch1, R=ch2)
  if (gain_r != 1.0f || gain_b != 1.0f) {
    std::vector<cv::Mat> ch(3);
    cv::split(flt, ch);
    ch[2] *= gain_r;
    ch[0] *= gain_b;
    cv::merge(ch, flt);
  }

  // CCM interpolated at this frame's CCT (only when tuning is loaded and CCT known)
  if (tuning && tuning->has_ccm && cct > 0.0f) {
    float m[9];
    interpolateCcm(*tuning, cct, m);
    cv::Mat M = ccmToBgrMat(m);
    cv::transform(flt, flt, M);
  }

  // Clamp and convert back to uint8
  cv::threshold(flt, flt, 255.0, 255.0, cv::THRESH_TRUNC);
  cv::threshold(flt, flt,   0.0,   0.0, cv::THRESH_TOZERO);
  flt.convertTo(bgr, CV_8UC3);

  // ── Gamma ─────────────────────────────────────────────────────────────────
  // Prefer the sensor-specific piecewise curve from the tuning file;
  // fall back to a power-law curve if no tuning is available.
  bool apply_gamma = true;
  uint8_t lut[256];

  if (tuning && tuning->has_gamma) {
    buildTuningGammaLut(*tuning, lut);
  } else if (cfg.gamma > 0.0f && cfg.gamma != 1.0f) {
    const double inv = 1.0 / cfg.gamma;
    for (int i = 0; i < 256; ++i)
      lut[i] = static_cast<uint8_t>(std::round(std::pow(i / 255.0, inv) * 255.0));
  } else {
    apply_gamma = false;
  }

  if (apply_gamma) {
    cv::Mat lut_mat(1, 256, CV_8UC1, lut);
    cv::LUT(bgr, lut_mat, bgr);
  }

  if (cfg.rotate180) cv::rotate(bgr, bgr, cv::ROTATE_180);

  if (!cv::imwrite(output.string(), bgr)) {
    std::cerr << "Failed to write: " << output << "\n";
    return false;
  }
  return true;
}

// ─── CLI ─────────────────────────────────────────────────────────────────────

static void printUsage(const char* prog) {
  std::cerr
      << "Usage:\n"
      << "  " << prog << " [options] <input.raw> <output.[png|jpg|ppm|tiff]>\n"
      << "  " << prog << " [options] --batch <input_dir> <output_dir>\n"
      << "\nOptions:\n"
      << "  --width   W   Sensor pixel width  (default: 2328)\n"
      << "  --height  H   Sensor pixel height (default: 1748)\n"
      << "  --stride  S   Bytes per row       (default: 2368)\n"
      << "  --awb-r   R   Red  AWB gain       (default: from sidecar or 1.0)\n"
      << "  --awb-b   B   Blue AWB gain       (default: from sidecar or 1.0)\n"
      << "  --quality Q   bilinear (default) | ea | vng\n"
      << "  --tuning  F   libcamera tuning JSON (enables CCM + sensor gamma curve)\n"
      << "  --rotate180   Rotate 180 degrees\n"
      << "  --black   N   Black level in 16-bit space (default: 4096)\n"
      << "  --white   N   White level in 16-bit space (default: 65535)\n"
      << "  --autostretch Per-frame min/max normalisation\n"
      << "  --gamma   G   Fallback power-law gamma (default: 2.2; 0 disables; ignored when --tuning used)\n"
      << "  --ext     E   Output extension for --batch (default: png)\n";
}

int main(int argc, char* argv[]) {
  if (argc < 3) { printUsage(argv[0]); return 1; }

  Config cfg;
  std::string batch_ext = "png";
  std::vector<std::string> positional;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto nextStr   = [&]() -> std::string { if (i+1 >= argc) { std::cerr << "Missing value for " << arg << "\n"; std::exit(1); } return argv[++i]; };
    auto nextFloat = [&]() { return std::stof(nextStr()); };
    auto nextInt   = [&]() { return std::stoi(nextStr()); };

    if      (arg == "--width")       cfg.width       = nextInt();
    else if (arg == "--height")      cfg.height      = nextInt();
    else if (arg == "--stride")      cfg.stride      = nextInt();
    else if (arg == "--awb-r")     { cfg.gain_r      = nextFloat(); cfg.gains_from_cli = true; }
    else if (arg == "--awb-b")     { cfg.gain_b      = nextFloat(); cfg.gains_from_cli = true; }
    else if (arg == "--quality")     cfg.quality     = nextStr();
    else if (arg == "--tuning")      cfg.tuning_path = nextStr();
    else if (arg == "--rotate180")   cfg.rotate180   = true;
    else if (arg == "--black")       cfg.black_level = nextInt();
    else if (arg == "--white")       cfg.white_level = nextInt();
    else if (arg == "--autostretch") cfg.auto_stretch = true;
    else if (arg == "--gamma")       cfg.gamma       = nextFloat();
    else if (arg == "--batch")       cfg.batch       = true;
    else if (arg == "--ext")         batch_ext       = nextStr();
    else if (arg.substr(0, 2) == "--") { std::cerr << "Unknown option: " << arg << "\n"; return 1; }
    else positional.push_back(arg);
  }

  if (positional.size() < 2) { printUsage(argv[0]); return 1; }

  // Load tuning once (shared across all frames in batch mode)
  std::optional<TuningData> tuning_storage;
  const TuningData* tuning = nullptr;
  if (!cfg.tuning_path.empty()) {
    tuning_storage = loadTuningData(cfg.tuning_path);
    tuning = &*tuning_storage;
    std::cout << "Tuning: "
              << (tuning->has_ccm   ? std::to_string(tuning->ccms.size()) + " CCM entries" : "no CCM")
              << ", "
              << (tuning->has_gamma ? std::to_string(tuning->gamma_x.size()) + "-point gamma curve" : "no gamma curve")
              << "\n";
  }

  if (cfg.batch) {
    fs::path in_dir  = positional[0];
    fs::path out_dir = positional[1];
    if (!fs::is_directory(in_dir)) { std::cerr << "Not a directory: " << in_dir << "\n"; return 1; }
    fs::create_directories(out_dir);

    // Load all per-camera AWB sidecars
    std::map<uint32_t, std::map<uint32_t, AwbRecord>> all_awb;
    for (const auto& e : fs::directory_iterator(in_dir)) {
      const std::string name = e.path().filename().string();
      if (name.substr(0, 3) == "cam" && name.find("_awb.bin") != std::string::npos) {
        auto us = name.find('_');
        try {
          uint32_t cid = static_cast<uint32_t>(std::stoul(name.substr(3, us - 3)));
          all_awb[cid] = loadAwbSidecar(e.path());
          std::cout << "Loaded " << all_awb[cid].size()
                    << " AWB records for camera " << cid << "\n";
        } catch (...) {}
      }
    }

    int ok = 0, fail = 0;
    for (const auto& e : fs::directory_iterator(in_dir)) {
      if (e.path().extension() != ".raw") continue;
      const std::string stem = e.path().stem().string();
      auto parsed = parseRawStem(stem);
      if (!parsed) { std::cerr << "Skipping: " << e.path().filename() << "\n"; continue; }
      auto [cam_id, frame_id] = *parsed;

      const std::map<uint32_t, AwbRecord>* awb_map = nullptr;
      auto it = all_awb.find(cam_id);
      if (it != all_awb.end()) awb_map = &it->second;

      fs::path out_file = out_dir / (stem + "." + batch_ext);
      if (debayerFile(e.path(), out_file, cfg, awb_map, frame_id, tuning)) {
        ++ok; std::cout << "  OK  " << stem << "\n";
      } else { ++fail; }
    }
    std::cout << "\nDone: " << ok << " OK, " << fail << " failed.\n";
    return fail > 0 ? 1 : 0;

  } else {
    fs::path input  = positional[0];
    fs::path output = positional[1];

    std::map<uint32_t, AwbRecord> awb_map;
    std::optional<uint32_t> frame_id_opt;
    if (!cfg.gains_from_cli) {
      auto parsed = parseRawStem(input.stem().string());
      if (parsed) {
        auto [cam_id, frame_id] = *parsed;
        frame_id_opt = frame_id;
        fs::path sidecar = input.parent_path() /
                           ("cam" + std::to_string(cam_id) + "_awb.bin");
        if (fs::exists(sidecar)) {
          awb_map = loadAwbSidecar(sidecar);
          std::cout << "Loaded AWB sidecar (" << awb_map.size() << " records)\n";
        }
      }
    }

    if (!debayerFile(input, output, cfg,
                     awb_map.empty() ? nullptr : &awb_map, frame_id_opt, tuning))
      return 1;

    std::cout << "Saved: " << output << "\n";
    return 0;
  }
}
