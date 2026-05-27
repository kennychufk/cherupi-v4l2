#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "frame_saver.hpp"
#include "frame_sink.hpp"
#include "stream_manager.hpp"
#include "types.hpp"

namespace {

// Captures each send() call as a separate message so tests can inspect the
// chunk stream independently of any transport.
class RecordingSink : public FrameSink {
 public:
  std::vector<std::vector<uint8_t>> messages;
  size_t buffered = 0;

  bool send(const uint8_t* data, size_t len) override {
    messages.emplace_back(data, data + len);
    return true;
  }
  size_t bufferedAmount() const override { return buffered; }
};

FrameData makeSyntheticFrame(size_t total_bytes, uint32_t cam_id = 0) {
  FrameData f;
  f.camera_id = cam_id;
  f.frame_id = 7;
  f.width = 1920;
  f.height = 1080;
  f.bytes_per_line = 1920;
  f.pixel_format = 0x32315559;  // 'YU12'
  f.data.resize(total_bytes);
  for (size_t i = 0; i < total_bytes; ++i) {
    f.data[i] = static_cast<uint8_t>(i & 0xFF);
  }
  return f;
}

TEST(StreamChunkingTest, ChunkedFrameProducesHeaderPlusChunkCount) {
  CameraManager dummy_mgr;
  FrameSaver dummy_saver;
  StreamManager sm(&dummy_mgr, &dummy_saver);

  RecordingSink sink;
  sm.setSink(&sink);

  const size_t payload = 1u << 20;  // 1 MiB
  FrameData frame = makeSyntheticFrame(payload);
  ASSERT_TRUE(sm.sendFrameForTest(frame, /*camera_id=*/0));

  const size_t expected_chunks =
      (payload + StreamManager::CHUNK_SIZE - 1) / StreamManager::CHUNK_SIZE;
  EXPECT_EQ(sink.messages.size(), 1 + expected_chunks);

  // Message 0: ChunkStartMarker + ChunkHeader. No corner block when the save
  // mode is NONE (the default for `dummy_saver`), so the message is exactly
  // marker + header bytes.
  ASSERT_EQ(sink.messages[0].size(),
            sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkStartMarker marker{};
  std::memcpy(&marker, sink.messages[0].data(), sizeof(marker));
  EXPECT_EQ(marker.magic, 0x4348554Eu);
  EXPECT_EQ(marker.version, 5u);

  ChunkHeader header{};
  std::memcpy(&header, sink.messages[0].data() + sizeof(marker),
              sizeof(header));
  EXPECT_EQ(header.total_chunks, expected_chunks);
  EXPECT_EQ(header.total_size, payload);
  EXPECT_EQ(header.width, 1920u);
  EXPECT_EQ(header.height, 1080u);
  EXPECT_EQ(header.corner_block_size, 0u);
  EXPECT_EQ(header.num_corner_sets, 0u);

  // Subsequent messages are ChunkData packets.
  for (size_t i = 0; i < expected_chunks; ++i) {
    ChunkData cd{};
    ASSERT_GE(sink.messages[i + 1].size(), sizeof(ChunkData));
    std::memcpy(&cd, sink.messages[i + 1].data(), sizeof(cd));
    EXPECT_EQ(cd.magic, 0x43484E4Bu);
    EXPECT_EQ(cd.chunk_index, i);
    EXPECT_EQ(cd.frame_uuid, header.frame_uuid);

    size_t expected_size =
        std::min<size_t>(StreamManager::CHUNK_SIZE, payload - i * StreamManager::CHUNK_SIZE);
    EXPECT_EQ(cd.chunk_size, expected_size);
    EXPECT_EQ(sink.messages[i + 1].size(), sizeof(ChunkData) + expected_size);
  }

  sm.clearSink();
}

TEST(StreamChunkingTest, LastChunkCarriesRemainderBytes) {
  CameraManager dummy_mgr;
  FrameSaver dummy_saver;
  StreamManager sm(&dummy_mgr, &dummy_saver);

  RecordingSink sink;
  sm.setSink(&sink);

  // Pick a size that is not a multiple of CHUNK_SIZE.
  const size_t payload = StreamManager::CHUNK_SIZE * 3 + 123;
  FrameData frame = makeSyntheticFrame(payload);
  ASSERT_TRUE(sm.sendFrameForTest(frame, 0));

  ChunkData last{};
  std::memcpy(&last, sink.messages.back().data(), sizeof(last));
  EXPECT_EQ(last.chunk_size, 123u);
  EXPECT_EQ(sink.messages.back().size(), sizeof(ChunkData) + 123u);

  sm.clearSink();
}

TEST(StreamChunkingTest, HeaderOnlyModeSendsOnlyMarkerPlusHeader) {
  CameraManager dummy_mgr;
  FrameSaver dummy_saver;
  StreamManager sm(&dummy_mgr, &dummy_saver);

  RecordingSink sink;
  sm.setSink(&sink);
  sm.setHeaderOnlyMode(true);

  FrameData frame = makeSyntheticFrame(1u << 20);
  ASSERT_TRUE(sm.sendFrameForTest(frame, 0));

  ASSERT_EQ(sink.messages.size(), 1u);
  ASSERT_EQ(sink.messages[0].size(),
            sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkHeader header{};
  std::memcpy(&header,
              sink.messages[0].data() + sizeof(ChunkStartMarker),
              sizeof(header));
  EXPECT_EQ(header.total_chunks, 0u);
  EXPECT_EQ(header.total_size, 0u);
  // Geometry is still preserved in header-only mode.
  EXPECT_EQ(header.width, frame.width);
  EXPECT_EQ(header.height, frame.height);

  sm.clearSink();
}

TEST(StreamChunkingTest, ChunkHeaderEmitsZeroCornerBlockOnNoDetection) {
  // CHECKERBOARD2X2 over a uniform-gray frame: the saver caches an empty set
  // list for (camera_id, frame_id), and the streamer must encode that as
  // num_corner_sets=0 with no bytes appended to the header message.
  CameraManager dummy_mgr;
  FrameSaver saver;
  SaveConfig cfg;
  cfg.mode = SaveMode::CHECKERBOARD2X2;
  cfg.checkerboard_full_res_detection = true;
  cfg.checkerboard_num_threads = 4;
  cfg.writer_threads = 1;
  cfg.output_dir = ".";
  saver.configure(cfg);
  saver.start();

  FrameData detection_frame;
  detection_frame.camera_id = 2;
  detection_frame.frame_id = 99;
  detection_frame.width = 480;
  detection_frame.height = 360;
  detection_frame.bytes_per_line = 480;
  detection_frame.pixel_format = 0x32315559;
  detection_frame.data.assign(static_cast<size_t>(480) * 360 * 3 / 2, 128);
  saver.saveFrame(detection_frame);

  StreamManager sm(&dummy_mgr, &saver);
  RecordingSink sink;
  sm.setSink(&sink);

  // Stream the same (camera_id, frame_id) so the cache hit lands.
  FrameData stream_frame = detection_frame;
  ASSERT_TRUE(sm.sendFrameForTest(stream_frame, /*camera_id=*/2));

  // No board → empty corner set list → header reflects 0 sets with no extra
  // bytes appended to the header message.
  ASSERT_GE(sink.messages.size(), 1u);
  ASSERT_EQ(sink.messages[0].size(),
            sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkHeader header{};
  std::memcpy(&header,
              sink.messages[0].data() + sizeof(ChunkStartMarker),
              sizeof(header));
  EXPECT_EQ(header.corner_block_size, 0u);
  EXPECT_EQ(header.num_corner_sets, 0u);

  sm.clearSink();
  saver.stop();
}

TEST(StreamChunkingTest, ChunkHeaderAppendsCornerBlockForDetectingFrame) {
  // End-to-end: save mode is CHECKERBOARD2X2, a YUV frame with the fixture
  // pasted into the top-left quadrant gets saveFrame'd (populating the
  // detection cache), then the streamer sends the same frame and we read
  // back the CornerSetHeader + corner coordinates.
  std::ifstream f(std::string(CHERUPI_TEST_FIXTURES_DIR) +
                      "/checkerboard_8x11.pgm",
                  std::ios::binary);
  ASSERT_TRUE(f.is_open());
  std::string magic;
  f >> magic;
  ASSERT_EQ(magic, "P5");
  int board_w, board_h, maxval;
  f >> board_w >> board_h >> maxval;
  f.get();
  std::vector<uint8_t> board(static_cast<size_t>(board_w) * board_h);
  f.read(reinterpret_cast<char*>(board.data()), board.size());

  // Frame size so the board fits in one full-res quadrant.
  const uint32_t W = static_cast<uint32_t>(board_w) * 2;
  const uint32_t H = static_cast<uint32_t>(board_h) * 2;

  FrameData frame;
  frame.camera_id = 5;
  frame.frame_id = 11;
  frame.width = W;
  frame.height = H;
  frame.bytes_per_line = W;
  frame.pixel_format = 0x32315559;
  frame.data.assign(static_cast<size_t>(W) * H * 3 / 2, 128);
  for (int y = 0; y < board_h; ++y) {
    std::memcpy(frame.data.data() + static_cast<size_t>(y) * W,
                board.data() + static_cast<size_t>(y) * board_w,
                static_cast<size_t>(board_w));
  }

  SaveConfig cfg;
  cfg.mode = SaveMode::CHECKERBOARD2X2;
  cfg.checkerboard_cols = 11;
  cfg.checkerboard_rows = 8;
  cfg.checkerboard_full_res_detection = true;
  cfg.checkerboard_num_threads = 4;
  cfg.writer_threads = 1;
  cfg.output_dir = ".";

  FrameSaver saver;
  saver.configure(cfg);
  saver.start();
  saver.saveFrame(frame);

  CameraManager dummy_mgr;
  StreamManager sm(&dummy_mgr, &saver);
  RecordingSink sink;
  sm.setSink(&sink);

  ASSERT_TRUE(sm.sendFrameForTest(frame, frame.camera_id));
  ASSERT_GE(sink.messages.size(), 1u);

  // Parse marker + header from message 0.
  ChunkStartMarker marker{};
  std::memcpy(&marker, sink.messages[0].data(), sizeof(marker));
  EXPECT_EQ(marker.version, 5u);

  ChunkHeader header{};
  std::memcpy(&header,
              sink.messages[0].data() + sizeof(ChunkStartMarker),
              sizeof(header));
  ASSERT_EQ(header.num_corner_sets, 1u);
  const uint32_t expected_corners = 11u * 8u;  // rows × cols
  ASSERT_EQ(header.corner_block_size,
            sizeof(CornerSetHeader) + expected_corners * 2 * sizeof(float));
  ASSERT_EQ(sink.messages[0].size(),
            sizeof(ChunkStartMarker) + sizeof(ChunkHeader) +
                header.corner_block_size);

  // CornerSetHeader sits immediately after ChunkHeader.
  CornerSetHeader sh{};
  std::memcpy(&sh,
              sink.messages[0].data() + sizeof(ChunkStartMarker) +
                  sizeof(ChunkHeader),
              sizeof(sh));
  EXPECT_EQ(sh.set_id, 0u);          // top-left quadrant (row=0, col=0)
  EXPECT_EQ(sh.flags & 0x01, 0x01);  // full-frame Y-plane coords
  EXPECT_EQ(sh.num_corners, expected_corners);

  // Spot-check the first corner: it must lie inside the top-left quadrant
  // [0, W/2) × [0, H/2).
  float xy[2];
  std::memcpy(xy,
              sink.messages[0].data() + sizeof(ChunkStartMarker) +
                  sizeof(ChunkHeader) + sizeof(CornerSetHeader),
              sizeof(xy));
  EXPECT_GT(xy[0], 0.0f);
  EXPECT_LT(xy[0], static_cast<float>(W) / 2.0f);
  EXPECT_GT(xy[1], 0.0f);
  EXPECT_LT(xy[1], static_cast<float>(H) / 2.0f);

  sm.clearSink();
  saver.stop();
}

TEST(StreamChunkingTest, ClearingSinkResetsRateControl) {
  CameraManager dummy_mgr;
  FrameSaver dummy_saver;
  StreamManager sm(&dummy_mgr, &dummy_saver);

  RecordingSink sink;
  sm.setSink(&sink);

  // Deliver a small frame so the chunk loop runs at least once.
  FrameData frame = makeSyntheticFrame(StreamManager::CHUNK_SIZE / 2);
  ASSERT_TRUE(sm.sendFrameForTest(frame, 0));

  sm.clearSink();
  // Another send with no sink should fail cleanly rather than crash.
  EXPECT_FALSE(sm.sendFrameForTest(frame, 0));
}

}  // namespace
