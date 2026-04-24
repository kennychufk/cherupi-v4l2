#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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
