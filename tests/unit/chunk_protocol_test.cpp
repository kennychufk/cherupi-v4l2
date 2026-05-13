#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "types.hpp"

namespace {

TEST(ChunkProtocolTest, StructSizesMatchWireFormat) {
  // Wire format is packed; any silent drift in these sizes breaks every
  // existing client. ChunkHeader grew from 40→52 bytes at protocol v3 (added
  // timestamp_us uint64 + frame_duration_us uint32).
  EXPECT_EQ(sizeof(ChunkStartMarker), 8u);
  EXPECT_EQ(sizeof(ChunkHeader), 52u);
  EXPECT_EQ(sizeof(ChunkData), 16u);
}

TEST(ChunkProtocolTest, StartMarkerByteLayout) {
  ChunkStartMarker marker{};
  // 'CHUN' little-endian = 4E 55 48 43, version 3 = 03 00 00 00
  const uint8_t expected[8] = {0x4E, 0x55, 0x48, 0x43,
                               0x03, 0x00, 0x00, 0x00};
  EXPECT_EQ(0, std::memcmp(&marker, expected, sizeof(expected)));
}

TEST(ChunkProtocolTest, ChunkDataMagicAndFieldOffsets) {
  ChunkData cd{};
  cd.frame_uuid = 0x11223344;
  cd.chunk_index = 0xDEADBEEF;
  cd.chunk_size = 0x00010000;

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cd);

  // 'CHNK' little-endian = 4B 4E 48 43
  EXPECT_EQ(bytes[0], 0x4B);
  EXPECT_EQ(bytes[1], 0x4E);
  EXPECT_EQ(bytes[2], 0x48);
  EXPECT_EQ(bytes[3], 0x43);

  // frame_uuid at offset 4 (LE of 0x11223344)
  EXPECT_EQ(bytes[4], 0x44);
  EXPECT_EQ(bytes[5], 0x33);
  EXPECT_EQ(bytes[6], 0x22);
  EXPECT_EQ(bytes[7], 0x11);

  // chunk_index at offset 8, chunk_size at offset 12
  uint32_t idx, size;
  std::memcpy(&idx, bytes + 8, 4);
  std::memcpy(&size, bytes + 12, 4);
  EXPECT_EQ(idx, 0xDEADBEEFu);
  EXPECT_EQ(size, 0x00010000u);
}

TEST(ChunkProtocolTest, ChunkHeaderFieldOffsetsAreStable) {
  // Spot check a few critical field offsets so rearranging the struct without
  // updating this test is impossible.
  ChunkHeader h{};
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&h);

  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.frame_uuid) - base, 0);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.frame_id) - base, 4);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.camera_id) - base, 8);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.total_chunks) - base, 12);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.total_size) - base, 16);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.bytes_per_line) - base, 20);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.width) - base, 24);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.height) - base, 28);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.pixel_format) - base, 32);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.frames_saved) - base, 36);
  // v3 additions
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.timestamp_us) - base, 40);
  EXPECT_EQ(reinterpret_cast<const uint8_t*>(&h.frame_duration_us) - base, 48);
}

}  // namespace
