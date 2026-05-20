#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "frame_saver.hpp"
#include "types.hpp"

namespace {

namespace fs = std::filesystem;

// Synthesize a tiny YUV-style frame. Content is enough to round-trip bytes to
// disk and read them back.
FrameData makeFrame(uint32_t camera_id, uint32_t frame_id, size_t size = 16,
                    uint8_t fill_byte = 0) {
  FrameData f;
  f.camera_id = camera_id;
  f.frame_id = frame_id;
  f.width = 4;
  f.height = 4;
  f.bytes_per_line = 4;
  f.pixel_format = 0x32315559;  // 'YU12' (YUV420)
  uint8_t marker = fill_byte ? fill_byte : static_cast<uint8_t>(frame_id);
  f.data.assign(size, marker);
  return f;
}

class FrameSaverTempDir : public ::testing::Test {
 protected:
  fs::path dir;

  void SetUp() override {
    // Use an on-disk scratch directory (provided by CMake) rather than
    // fs::temp_directory_path(), which on some hosts (e.g. Raspberry Pi) is
    // tmpfs and does not reliably support the O_DIRECT writes BATCH mode
    // issues.
    fs::path scratch_base(CHERUPI_TEST_SCRATCH_DIR);
    fs::create_directories(scratch_base);
    dir = scratch_base / fs::path("cherupi_saver_" +
                                  std::to_string(::getpid()) + "_" +
                                  std::to_string(counter()));
    fs::create_directories(dir);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir, ec);
  }

 private:
  static int counter() {
    static std::atomic<int> n{0};
    return ++n;
  }
};

TEST_F(FrameSaverTempDir, NoneModeDoesNothing) {
  SaveConfig cfg;
  cfg.mode = SaveMode::NONE;
  cfg.output_dir = dir.string();

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();

  for (int i = 0; i < 3; ++i) saver.saveFrame(makeFrame(0, i));

  saver.stop();
  EXPECT_EQ(saver.getFramesSaved(), 0u);
  EXPECT_FALSE(saver.isEnabled());

  // No files should be created under `dir`.
  int count = 0;
  for ([[maybe_unused]] auto& _ : fs::directory_iterator(dir)) ++count;
  EXPECT_EQ(count, 0);
}

TEST_F(FrameSaverTempDir, BufferModeFlushesOnRequest) {
  SaveConfig cfg;
  cfg.mode = SaveMode::BUFFER;
  cfg.output_dir = dir.string();

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();
  ASSERT_TRUE(saver.isEnabled());

  for (uint32_t i = 0; i < 3; ++i) saver.saveFrame(makeFrame(0, i));

  // Before flush, nothing on disk.
  int pre_count = 0;
  for ([[maybe_unused]] auto& _ : fs::directory_iterator(dir)) ++pre_count;
  EXPECT_EQ(pre_count, 0);

  saver.flushBufferedFrames();

  EXPECT_EQ(saver.getFramesSaved(), 3u);
  for (uint32_t i = 0; i < 3; ++i) {
    fs::path expected = dir / ("cam0-" + std::to_string(i) + ".yuv");
    ASSERT_TRUE(fs::exists(expected)) << expected;
    std::ifstream f(expected, std::ios::binary);
    std::vector<char> contents((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());
    ASSERT_EQ(contents.size(), 16u);
    EXPECT_EQ(static_cast<uint8_t>(contents[0]), i);
  }

  EXPECT_EQ(saver.getFramesSavedForCamera(0), 3u);
  saver.stop();
}

TEST_F(FrameSaverTempDir, BatchModeWritesThroughWriterThreads) {
  // BATCH mode uses O_DIRECT; on some filesystems the alignment pads bytes,
  // which is acceptable. Here we only check that a file per frame appears.
  SaveConfig cfg;
  cfg.mode = SaveMode::BATCH;
  cfg.output_dir = dir.string();
  cfg.writer_threads = 2;

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();

  const int kFrames = 10;
  for (int i = 0; i < kFrames; ++i) {
    saver.saveFrame(makeFrame(/*camera_id=*/1, /*frame_id=*/i, 4096));
  }
  saver.stop();  // joins writer threads after queue drains

  EXPECT_EQ(saver.getFramesSaved(), static_cast<size_t>(kFrames));
  EXPECT_EQ(saver.getFramesSavedForCamera(1), static_cast<uint32_t>(kFrames));

  for (int i = 0; i < kFrames; ++i) {
    fs::path expected = dir / ("cam1-" + std::to_string(i) + ".yuv");
    EXPECT_TRUE(fs::exists(expected)) << expected;
  }
}

TEST_F(FrameSaverTempDir, CountersIncrementPerCamera) {
  SaveConfig cfg;
  cfg.mode = SaveMode::BUFFER;
  cfg.output_dir = dir.string();

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();

  saver.saveFrame(makeFrame(0, 0));
  saver.saveFrame(makeFrame(0, 1));
  saver.saveFrame(makeFrame(1, 0));
  saver.flushBufferedFrames();

  EXPECT_EQ(saver.getFramesSavedForCamera(0), 2u);
  EXPECT_EQ(saver.getFramesSavedForCamera(1), 1u);
  EXPECT_EQ(saver.getFramesSavedForCamera(99), 0u);

  saver.stop();
}

// Load the checkerboard PGM fixture and return (data, width, height).
struct Pgm {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;
};
static Pgm loadCheckerboardFixture() {
  std::ifstream f(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                      "/checkerboard_8x11.pgm",
                  std::ios::binary);
  std::string magic;
  f >> magic;
  Pgm out;
  if (magic != "P5") return out;
  int maxval;
  f >> out.width >> out.height >> maxval;
  f.get();
  out.data.resize(static_cast<size_t>(out.width) * out.height);
  f.read(reinterpret_cast<char*>(out.data.data()), out.data.size());
  return out;
}

// Synthesize a YUV420 frame whose Y plane is `2*board_w x 2*board_h` and
// optionally has the supplied board pasted into its top-left quadrant. UV
// planes are filled with mid-gray (irrelevant for detection).
static FrameData makeYuvFrameWithOptionalBoard(uint32_t camera_id,
                                                uint32_t frame_id,
                                                const Pgm& board,
                                                bool paste) {
  FrameData frame;
  frame.camera_id = camera_id;
  frame.frame_id = frame_id;
  frame.width = static_cast<uint32_t>(board.width) * 2;
  frame.height = static_cast<uint32_t>(board.height) * 2;
  frame.bytes_per_line = frame.width;
  frame.pixel_format = 0x32315559;  // 'YU12' (YUV420)
  const size_t y_bytes = static_cast<size_t>(frame.width) * frame.height;
  const size_t uv_bytes = y_bytes / 2;
  frame.data.assign(y_bytes + uv_bytes, 128);
  if (paste) {
    for (int y = 0; y < board.height; ++y) {
      std::memcpy(frame.data.data() + static_cast<size_t>(y) * frame.width,
                  board.data.data() + static_cast<size_t>(y) * board.width,
                  static_cast<size_t>(board.width));
    }
  }
  return frame;
}

TEST_F(FrameSaverTempDir, Checkerboard2x2SavesWhenAnyQuadrantHasBoard) {
  // Fixture occupies the top-left quadrant of the Y plane; with
  // full_res_detection=true the saver splits the full Y into 4 W/2 x H/2
  // quadrants and detection should hit on the top-left only.
  Pgm board = loadCheckerboardFixture();
  ASSERT_GT(board.width, 0);
  FrameData frame =
      makeYuvFrameWithOptionalBoard(/*camera_id=*/0, /*frame_id=*/42, board,
                                    /*paste=*/true);

  SaveConfig cfg;
  cfg.mode = SaveMode::CHECKERBOARD2X2;
  cfg.output_dir = dir.string();
  cfg.checkerboard_cols = 11;
  cfg.checkerboard_rows = 8;
  cfg.checkerboard_full_res_detection = true;
  cfg.checkerboard_num_threads = 4;
  cfg.writer_threads = 1;

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();
  saver.saveFrame(frame);
  saver.stop();

  EXPECT_EQ(saver.getFramesChecked(), 1u);
  EXPECT_EQ(saver.getCheckerboardsDetected(), 1u);
  EXPECT_EQ(saver.getFramesSaved(), 1u);
  EXPECT_TRUE(fs::exists(dir / "cam0-42.yuv"));
}

TEST_F(FrameSaverTempDir, Checkerboard2x2SkipsWhenNoQuadrantHasBoard) {
  Pgm board = loadCheckerboardFixture();
  ASSERT_GT(board.width, 0);
  // Same frame geometry as above, but no board pasted — uniform mid-gray Y.
  FrameData frame =
      makeYuvFrameWithOptionalBoard(/*camera_id=*/0, /*frame_id=*/1, board,
                                    /*paste=*/false);

  SaveConfig cfg;
  cfg.mode = SaveMode::CHECKERBOARD2X2;
  cfg.output_dir = dir.string();
  cfg.checkerboard_full_res_detection = true;
  cfg.checkerboard_num_threads = 4;
  cfg.writer_threads = 1;

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();
  saver.saveFrame(frame);
  saver.stop();

  EXPECT_EQ(saver.getFramesChecked(), 1u);
  EXPECT_EQ(saver.getCheckerboardsDetected(), 0u);
  EXPECT_EQ(saver.getFramesSaved(), 0u);
  int count = 0;
  for ([[maybe_unused]] auto& _ : fs::directory_iterator(dir)) ++count;
  EXPECT_EQ(count, 0);
}

TEST_F(FrameSaverTempDir, Checkerboard2x2HonoursThreadPoolSize) {
  // num_threads=1 forces sequential evaluation. Result must still be
  // correct (any-quadrant OR).
  Pgm board = loadCheckerboardFixture();
  ASSERT_GT(board.width, 0);
  FrameData frame =
      makeYuvFrameWithOptionalBoard(/*camera_id=*/3, /*frame_id=*/7, board,
                                    /*paste=*/true);

  SaveConfig cfg;
  cfg.mode = SaveMode::CHECKERBOARD2X2;
  cfg.output_dir = dir.string();
  cfg.checkerboard_full_res_detection = true;
  cfg.checkerboard_num_threads = 1;
  cfg.writer_threads = 1;

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();
  saver.saveFrame(frame);
  saver.stop();

  EXPECT_EQ(saver.getCheckerboardsDetected(), 1u);
  EXPECT_TRUE(fs::exists(dir / "cam3-7.yuv"));
}

TEST(FrameSaverTimestampDir, PrependTimestampProducesPrefixedDirectoryName) {
  fs::path base = fs::path(CHERUPI_TEST_SCRATCH_DIR) /
                  ("cherupi_ts_" + std::to_string(::getpid()));
  fs::create_directories(CHERUPI_TEST_SCRATCH_DIR);
  fs::remove_all(base);
  fs::create_directory(base);

  SaveConfig cfg;
  cfg.mode = SaveMode::BATCH;
  cfg.output_dir = (base / "runs").string();
  cfg.prepend_timestamp_to_dir = true;
  cfg.writer_threads = 1;

  FrameSaver saver;
  saver.configure(cfg);

  const std::string& actual = saver.getActualOutputDir();
  // The actual directory must end in "-runs" and start with the base parent.
  ASSERT_GE(actual.size(), std::string("YYYYMMDD-HHMM-runs").size());
  EXPECT_EQ(actual.rfind("-runs"), actual.size() - 5);
  EXPECT_NE(actual.find(base.string()), std::string::npos);

  saver.start();
  saver.stop();

  std::error_code ec;
  fs::remove_all(base, ec);
}

}  // namespace
