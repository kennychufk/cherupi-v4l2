// Converter for YUV420 planar frames saved by FrameSaver.
//
// File conventions (from frame_saver.cpp):
//   YUV frames: <dir>/cam<ID>-<frame_id>.yuv
//
// YUV420 planar (I420) layout:
//   Y plane:  stride × height bytes
//   U plane:  (stride/2) × (height/2) bytes  (half resolution)
//   V plane:  (stride/2) × (height/2) bytes  (half resolution)
//   Total file size padded to 4096 bytes by O_DIRECT writing
//
// Usage:
//   yuv420_convert [options] <input.yuv> <output.[png|jpg|ppm|tiff|pgm]>
//   yuv420_convert [options] --batch <input_dir> <output_dir>
//
// A .pgm output (or --ext pgm in batch) writes a grayscale Portable Graymap
// (binary P5) straight from the Y (luma) plane — no YUV→RGB conversion and no
// chroma read, so it is both faster and lower-memory than the colour path.
//
// With --split2x2 each input frame yields four quadrant images named
// cam<quadrant_camera_id>-<frame_id>.<ext>, where quadrant_camera_id =
// camera_id * 4 + q (q numbered before any --rotate180: upper-left 0,
// upper-right 1, bottom-left 2, bottom-right 3) and camera_id/frame_id come
// from the input's cam<camera_id>-<frame_id>.yuv name.
//
// Options:
//   --width   W   Pixel width            (default: 2328)
//   --height  H   Pixel height           (default: 1748)
//   --stride  S   Y-plane bytes per row  (default: 2432, libcamera alignment)
//   --quality Q   JPEG quality           (default: 95)
//   --rotate180   Rotate output 180 degrees
//   --split2x2    Split each frame into 4 quadrant images (cam0..cam3-<stem>)
//   --ext     E   Output extension for --batch (default: png)
//   --jobs    N   Parallel workers for --batch (default: hardware concurrency)

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// ─── Config ──────────────────────────────────────────────────────────────────

struct Config {
  bool batch = false;
  int width = 2328;
  int height = 1748;
  int stride = 2432;  // libcamera Y-plane stride (128-byte aligned)
  int jpeg_quality = 95;
  bool rotate180 = false;
  bool split2x2 = false;  // split each frame into 4 quadrant images
  int jobs = 0;           // --batch worker threads; 0 = hardware concurrency
};

// ─── Filename helpers ─────────────────────────────────────────────────────────

// Parse "cam<ID>-<frame_id>" stem → (cam_id, frame_id)
static std::optional<std::pair<uint32_t, uint32_t>> parseYuvStem(
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

// ─── Core YUV420 conversion ───────────────────────────────────────────────────

// A .pgm output (case-insensitive) means grayscale Portable Graymap.
static bool wantsGrayscale(const fs::path& output) {
  std::string ext = output.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext == ".pgm";
}

// Decode an I420 .yuv file into a Mat cropped to width×height — single-channel
// grayscale when `grayscale` is set, otherwise BGR. NOT rotated, split, or
// written; callers handle those.
static bool decodeYuvFrame(const fs::path& input, const Config& cfg,
                           bool grayscale, cv::Mat& out) {
  // Calculate expected sizes
  size_t y_size = static_cast<size_t>(cfg.stride) * cfg.height;
  size_t u_size = static_cast<size_t>(cfg.stride / 2) * (cfg.height / 2);
  size_t v_size = u_size;
  size_t expected_data = y_size + u_size + v_size;

  std::ifstream f(input, std::ios::binary);
  if (!f) {
    std::cerr << "Cannot open: " << input << "\n";
    return false;
  }

  // Grayscale only needs the Y plane; colour needs the full I420 frame.
  // (The file may be padded to 4096 bytes by O_DIRECT writing, but we only ever
  // read the bytes we actually use.)
  const size_t read_size = grayscale ? y_size : expected_data;
  std::vector<uint8_t> yuv_data(read_size);
  if (!f.read(reinterpret_cast<char*>(yuv_data.data()), read_size)) {
    std::cerr << "Short read: " << input << " (expected " << read_size
              << " bytes)\n";
    return false;
  }

  if (grayscale) {
    // The Y (luma) plane IS the grayscale image — no YUV→RGB conversion needed,
    // which is the fastest possible path. View just the `width` valid columns
    // of each stride-byte row, then clone into a packed buffer: this drops the
    // stride padding and gives a contiguous Mat that outlives yuv_data.
    cv::Mat y_view(cfg.height, cfg.width, CV_8UC1, yuv_data.data(),
                   static_cast<size_t>(cfg.stride));
    out = y_view.clone();
    return true;
  }

  // Create a contiguous I420 buffer for OpenCV
  // cv::cvtColor with COLOR_YUV2BGR_I420 expects:
  //   - A single-channel mat of size (height*3/2, stride)
  //   - Y plane: rows [0, height)
  //   - U plane: rows [height, height + height/2) with stride/2 per row
  //   - V plane: rows [height + height/2, height*3/2) with stride/2 per row
  //
  // We need to pack the data into this layout, padding each row appropriately.
  std::vector<uint8_t> i420_buf(cfg.stride * cfg.height * 3 / 2);

  // Copy Y plane (stride × height, no packing needed if stride == width)
  uint8_t* dst = i420_buf.data();
  const uint8_t* src = yuv_data.data();
  for (int y = 0; y < cfg.height; ++y) {
    std::memcpy(dst, src, cfg.stride);
    dst += cfg.stride;
    src += cfg.stride;
  }

  // Copy U plane (stride/2 × height/2)
  for (int y = 0; y < cfg.height / 2; ++y) {
    std::memcpy(dst, src, cfg.stride / 2);
    dst += cfg.stride / 2;
    src += cfg.stride / 2;
  }

  // Copy V plane (stride/2 × height/2)
  for (int y = 0; y < cfg.height / 2; ++y) {
    std::memcpy(dst, src, cfg.stride / 2);
    dst += cfg.stride / 2;
    src += cfg.stride / 2;
  }

  // Create mat from I420 buffer: height*3/2 rows × stride columns
  cv::Mat i420_mat(cfg.height * 3 / 2, cfg.stride, CV_8UC1, i420_buf.data());

  // Convert I420 → BGR
  cv::Mat bgr;
  cv::cvtColor(i420_mat, bgr, cv::COLOR_YUV2BGR_I420);

  // Crop to actual image dimensions if stride > width
  if (cfg.stride > cfg.width) {
    cv::Rect roi(0, 0, cfg.width, cfg.height);
    bgr = bgr(roi).clone();
  }
  out = std::move(bgr);
  return true;
}

// Split `full` into four quadrants and write each as
// cam<quadrant_camera_id>-<frame_id>.<ext> alongside `base_output`, where
// quadrant_camera_id = camera_id * 4 + q and camera_id/frame_id are parsed
// from `input`'s cam<camera_id>-<frame_id>.yuv name. Quadrant ids are fixed
// by the ORIGINAL (pre-rotation) position: upper-left 0, upper-right 1,
// bottom-left 2, bottom-right 3. With --rotate180 each quadrant is rotated
// 180° individually — identical pixels to rotating the whole frame then
// re-tiling, but cheaper: cv::rotate already yields a contiguous,
// directly-encodable buffer (no extra clone).
static bool writeQuadrants(const cv::Mat& full, const fs::path& input,
                           const fs::path& base_output, const Config& cfg) {
  auto parsed = parseYuvStem(input.stem().string());
  if (!parsed) {
    std::cerr << "Cannot parse cam<ID>-<frame_id> from input: " << input
              << "\n";
    return false;
  }
  const uint32_t camera_id = parsed->first;
  const uint32_t frame_id = parsed->second;

  const fs::path dir = base_output.parent_path();
  const std::string ext = base_output.extension().string();  // includes the dot

  const int wl = full.cols / 2, ht = full.rows / 2;    // left/top extents
  const int wr = full.cols - wl, hb = full.rows - ht;  // right/bottom (odd-safe)
  const cv::Rect quad_rects[4] = {
      cv::Rect(0, 0, wl, ht),    // 0 upper-left
      cv::Rect(wl, 0, wr, ht),   // 1 upper-right
      cv::Rect(0, ht, wl, hb),   // 2 bottom-left
      cv::Rect(wl, ht, wr, hb),  // 3 bottom-right
  };

  bool all_ok = true;
  for (int q = 0; q < 4; ++q) {
    cv::Mat quad;
    if (cfg.rotate180) {
      // rotate emits a fresh contiguous Mat — ready to encode, no extra clone.
      cv::rotate(full(quad_rects[q]), quad, cv::ROTATE_180);
    } else {
      quad = full(quad_rects[q]).clone();  // detach ROI into contiguous pixels
    }
    const uint32_t quadrant_camera_id =
        camera_id * 4 + static_cast<uint32_t>(q);
    fs::path out = dir / ("cam" + std::to_string(quadrant_camera_id) + "-" +
                          std::to_string(frame_id) + ext);
    if (!cv::imwrite(out.string(), quad)) {
      std::cerr << "Failed to write: " << out << "\n";
      all_ok = false;
    }
  }
  return all_ok;
}

static bool convertYuvFile(const fs::path& input, const fs::path& output,
                           const Config& cfg) {
  const bool grayscale = wantsGrayscale(output);

  cv::Mat full;
  if (!decodeYuvFrame(input, cfg, grayscale, full)) return false;

  // --split2x2: four quadrant files; any --rotate180 is applied per quadrant.
  if (cfg.split2x2) {
    return writeQuadrants(full, input, output, cfg);
  }

  // Single output: optional 180° rotation, then write.
  if (cfg.rotate180) {
    cv::rotate(full, full, cv::ROTATE_180);
  }
  if (!cv::imwrite(output.string(), full)) {
    std::cerr << "Failed to write: " << output << "\n";
    return false;
  }
  return true;
}

// ─── CLI ─────────────────────────────────────────────────────────────────────

static void printUsage(const char* prog) {
  std::cerr
      << "Usage:\n"
      << "  " << prog
      << " [options] <input.yuv> <output.[png|jpg|ppm|tiff|pgm]>\n"
      << "  " << prog << " [options] --batch <input_dir> <output_dir>\n"
      << "\nA .pgm output (or --ext pgm) writes a grayscale graymap from the Y "
         "plane.\n"
      << "--split2x2 writes 4 quadrant images "
         "cam<camera_id*4+q>-<frame_id>.<ext> per input.\n"
      << "\nOptions:\n"
      << "  --width   W   Pixel width             (default: 2328)\n"
      << "  --height  H   Pixel height            (default: 1748)\n"
      << "  --stride  S   Y-plane bytes per row   (default: 2432)\n"
      << "  --quality Q   JPEG quality            (default: 95)\n"
      << "  --rotate180   Rotate 180 degrees\n"
      << "  --split2x2    Split each frame into 4 quadrant images\n"
      << "  --ext     E   Output extension for --batch (default: png)\n"
      << "  --jobs    N   Parallel workers for --batch (default: all cores)\n";
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  Config cfg;
  std::string batch_ext = "png";
  std::vector<std::string> positional;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto nextStr = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << arg << "\n";
        std::exit(1);
      }
      return argv[++i];
    };
    auto nextInt = [&]() { return std::stoi(nextStr()); };

    if (arg == "--width")
      cfg.width = nextInt();
    else if (arg == "--height")
      cfg.height = nextInt();
    else if (arg == "--stride")
      cfg.stride = nextInt();
    else if (arg == "--quality")
      cfg.jpeg_quality = nextInt();
    else if (arg == "--rotate180")
      cfg.rotate180 = true;
    else if (arg == "--split2x2")
      cfg.split2x2 = true;
    else if (arg == "--batch")
      cfg.batch = true;
    else if (arg == "--ext")
      batch_ext = nextStr();
    else if (arg == "--jobs")
      cfg.jobs = nextInt();
    else if (arg.substr(0, 2) == "--") {
      std::cerr << "Unknown option: " << arg << "\n";
      return 1;
    } else {
      positional.push_back(arg);
    }
  }

  if (positional.size() < 2) {
    printUsage(argv[0]);
    return 1;
  }

  if (cfg.batch) {
    fs::path in_dir = positional[0];
    fs::path out_dir = positional[1];
    if (!fs::is_directory(in_dir)) {
      std::cerr << "Not a directory: " << in_dir << "\n";
      return 1;
    }
    fs::create_directories(out_dir);

    // Gather the conversion tasks up front (skipping non-.yuv / unparsable
    // names), then process them in parallel — every file is fully independent.
    struct Task {
      fs::path input;
      fs::path output;
      std::string stem;
    };
    std::vector<Task> tasks;
    for (const auto& e : fs::directory_iterator(in_dir)) {
      if (e.path().extension() != ".yuv") continue;
      const std::string stem = e.path().stem().string();
      if (!parseYuvStem(stem)) {
        std::cerr << "Skipping: " << e.path().filename() << "\n";
        continue;
      }
      tasks.push_back({e.path(), out_dir / (stem + "." + batch_ext), stem});
    }

    // Worker count: --jobs if given, else hardware concurrency, capped at the
    // number of tasks (and at least 1 so the loop stays well-formed).
    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 1;
    unsigned jobs = cfg.jobs > 0 ? static_cast<unsigned>(cfg.jobs) : hw;
    if (tasks.size() < jobs) jobs = static_cast<unsigned>(tasks.size());
    if (jobs == 0) jobs = 1;

    // Embarrassingly parallel: workers pull the next task index atomically and
    // report each file the moment it finishes (live progress, completion order)
    // rather than batching all prints after the join. A mutex keeps concurrent
    // lines from interleaving; atomic counters make the final tally race-free.
    std::atomic<size_t> next_idx{0};
    std::atomic<int> ok_count{0}, fail_count{0};
    std::mutex print_mtx;
    auto worker = [&]() {
      for (size_t i = next_idx.fetch_add(1); i < tasks.size();
           i = next_idx.fetch_add(1)) {
        bool ok = false;
        try {
          ok = convertYuvFile(tasks[i].input, tasks[i].output, cfg);
        } catch (const std::exception& ex) {
          // One bad frame must not abort the batch — an uncaught throw in a
          // worker (e.g. cv::imwrite on a bad codec) would std::terminate.
          std::lock_guard<std::mutex> lock(print_mtx);
          std::cerr << "Error on " << tasks[i].stem << ": " << ex.what() << "\n";
        }
        if (ok) {
          ++ok_count;
          std::lock_guard<std::mutex> lock(print_mtx);
          std::cout << "  OK  " << tasks[i].stem << "\n";
        } else {
          ++fail_count;
        }
      }
    };
    std::vector<std::thread> pool;
    pool.reserve(jobs);
    for (unsigned t = 0; t < jobs; ++t) pool.emplace_back(worker);
    for (auto& th : pool) th.join();

    const int ok = ok_count, fail = fail_count;
    std::cout << "\nDone: " << ok << " OK, " << fail << " failed (" << jobs
              << " thread" << (jobs == 1 ? "" : "s") << ").\n";
    return fail > 0 ? 1 : 0;

  } else {
    fs::path input = positional[0];
    fs::path output = positional[1];

    if (!convertYuvFile(input, output, cfg)) return 1;

    if (cfg.split2x2)
      std::cout << "Saved 4 quadrants for: " << output << "\n";
    else
      std::cout << "Saved: " << output << "\n";
    return 0;
  }
}
