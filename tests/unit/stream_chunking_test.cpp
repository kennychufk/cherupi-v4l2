#include <gtest/gtest.h>

#include <cstring>
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

  // Message 0: ChunkStartMarker (8 bytes) + ChunkHeader (40 bytes).
  ASSERT_EQ(sink.messages[0].size(),
            sizeof(ChunkStartMarker) + sizeof(ChunkHeader));

  ChunkStartMarker marker{};
  std::memcpy(&marker, sink.messages[0].data(), sizeof(marker));
  EXPECT_EQ(marker.magic, 0x4348554Eu);
  EXPECT_EQ(marker.version, 2u);

  ChunkHeader header{};
  std::memcpy(&header, sink.messages[0].data() + sizeof(marker),
              sizeof(header));
  EXPECT_EQ(header.total_chunks, expected_chunks);
  EXPECT_EQ(header.total_size, payload);
  EXPECT_EQ(header.width, 1920u);
  EXPECT_EQ(header.height, 1080u);

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
