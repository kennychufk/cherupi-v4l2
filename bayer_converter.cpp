#include "bayer_converter.h"

#include <arm_neon.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>

// Function declarations for internal use
void demosaic_half_res_neon(const uint16_t* bayer, uint8_t* grayscale,
                            int width, int height, int start_y, int end_y);
void demosaic_full_res_fast(const uint16_t* bayer, uint8_t* grayscale,
                            int width, int height, int start_y, int end_y);

BayerConverter::BayerConverter(const Config& cfg) : config(cfg) {
  out_width = config.full_res ? config.width : config.width / 2;
  out_height = config.full_res ? config.height : config.height / 2;
}

bool BayerConverter::process() {
  try {
    readInputFile();
    unpack10BitData();
    convertToGrayscale();

    if (config.save_ppm) {
      writePPM();
    }

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
}

void BayerConverter::readInputFile() {
  std::ifstream input(config.input_file, std::ios::binary | std::ios::ate);
  if (!input) {
    throw std::runtime_error("Cannot open input file");
  }

  size_t file_size = input.tellg();
  input.seekg(0);

  packed_data.resize(file_size);
  input.read(reinterpret_cast<char*>(packed_data.data()), file_size);

  std::cout << "Read " << file_size << " bytes\n";
  size_t expected_size = config.stride * config.height;
  std::cout << "Expected size: " << expected_size << " bytes\n";

  if (file_size != expected_size) {
    std::cerr << "Warning: File size mismatch!\n";
  }
}

void BayerConverter::unpack10BitData() {
  bayer_data.resize(config.width * config.height);

  std::cout << "Unpacking 10-bit data...\n";
  unpack_10bit_neon(packed_data.data(), bayer_data.data(), config.width,
                    config.height, config.stride);

  auto [min_it, max_it] =
      std::minmax_element(bayer_data.begin(), bayer_data.end());
  std::cout << "Data range: " << *min_it << " - " << *max_it << "\n";
}

void BayerConverter::convertToGrayscale() {
  grayscale_data.resize(out_width * out_height);

  std::cout << "Converting to grayscale ("
            << (config.full_res ? "full" : "half") << " resolution)...\n";

  demosaic_threaded(bayer_data.data(), grayscale_data.data(), config.width,
                    config.height, config.full_res, config.num_threads);

  std::cout << "Grayscale shape: " << out_width << "x" << out_height << "\n";
}

void BayerConverter::writePPM() {
  std::ofstream file(config.output_file, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Cannot open output file");
  }

  file << "P5\n" << out_width << " " << out_height << "\n255\n";
  file.write(reinterpret_cast<const char*>(grayscale_data.data()),
             out_width * out_height);

  std::cout << "Written PPM to " << config.output_file << "\n";
}

// Fast 10-bit unpacking using NEON intrinsics
void unpack_10bit_neon(const uint8_t* packed, uint16_t* unpacked, int width,
                       int height, int stride) {
  const int pixels_per_group = 4;
  const int bytes_per_group = 5;

  for (int y = 0; y < height; y++) {
    const uint8_t* line_ptr = packed + y * stride;
    uint16_t* out_ptr = unpacked + y * width;

    int x = 0;
    // Process 16 pixels at a time (4 groups of 4 pixels)
    for (; x + 15 < width; x += 16) {
      // Load 20 bytes (4 groups)
      uint8x16_t data = vld1q_u8(line_ptr);
      uint8x8_t extra = vld1_u8(line_ptr + 16);

      // Extract low 2 bits from every 5th byte
      uint8_t b4_0 = line_ptr[4];
      uint8_t b4_1 = line_ptr[9];
      uint8_t b4_2 = line_ptr[14];
      uint8_t b4_3 = line_ptr[19];

      // Unpack first group
      out_ptr[0] = (line_ptr[0] << 2) | ((b4_0 >> 6) & 0x03);
      out_ptr[1] = (line_ptr[1] << 2) | ((b4_0 >> 4) & 0x03);
      out_ptr[2] = (line_ptr[2] << 2) | ((b4_0 >> 2) & 0x03);
      out_ptr[3] = (line_ptr[3] << 2) | (b4_0 & 0x03);

      // Unpack second group
      out_ptr[4] = (line_ptr[5] << 2) | ((b4_1 >> 6) & 0x03);
      out_ptr[5] = (line_ptr[6] << 2) | ((b4_1 >> 4) & 0x03);
      out_ptr[6] = (line_ptr[7] << 2) | ((b4_1 >> 2) & 0x03);
      out_ptr[7] = (line_ptr[8] << 2) | (b4_1 & 0x03);

      // Unpack third group
      out_ptr[8] = (line_ptr[10] << 2) | ((b4_2 >> 6) & 0x03);
      out_ptr[9] = (line_ptr[11] << 2) | ((b4_2 >> 4) & 0x03);
      out_ptr[10] = (line_ptr[12] << 2) | ((b4_2 >> 2) & 0x03);
      out_ptr[11] = (line_ptr[13] << 2) | (b4_2 & 0x03);

      // Unpack fourth group
      out_ptr[12] = (line_ptr[15] << 2) | ((b4_3 >> 6) & 0x03);
      out_ptr[13] = (line_ptr[16] << 2) | ((b4_3 >> 4) & 0x03);
      out_ptr[14] = (line_ptr[17] << 2) | ((b4_3 >> 2) & 0x03);
      out_ptr[15] = (line_ptr[18] << 2) | (b4_3 & 0x03);

      line_ptr += 20;
      out_ptr += 16;
    }

    // Handle remaining pixels
    for (; x < width; x += pixels_per_group) {
      int group_idx = x / pixels_per_group;
      int base_idx = group_idx * bytes_per_group;

      if (base_idx + bytes_per_group > stride) break;

      const uint8_t* group_ptr = packed + y * stride + base_idx;
      uint8_t b4 = group_ptr[4];

      int pixels_to_unpack = std::min(4, width - x);

      if (pixels_to_unpack > 0)
        unpacked[y * width + x] = (group_ptr[0] << 2) | ((b4 >> 6) & 0x03);
      if (pixels_to_unpack > 1)
        unpacked[y * width + x + 1] = (group_ptr[1] << 2) | ((b4 >> 4) & 0x03);
      if (pixels_to_unpack > 2)
        unpacked[y * width + x + 2] = (group_ptr[2] << 2) | ((b4 >> 2) & 0x03);
      if (pixels_to_unpack > 3)
        unpacked[y * width + x + 3] = (group_ptr[3] << 2) | (b4 & 0x03);
    }
  }
}

// Half resolution demosaic with NEON optimization
void demosaic_half_res_neon(const uint16_t* bayer, uint8_t* grayscale,
                            int width, int height, int start_y, int end_y) {
  const float r_weight = 0.299f;
  const float g_weight = 0.587f;
  const float b_weight = 0.114f;
  const float scale = 255.0f / 1023.0f;

  // NEON constants
  float32x4_t r_weight_vec = vdupq_n_f32(r_weight);
  float32x4_t g_weight_vec = vdupq_n_f32(g_weight * 0.5f);
  float32x4_t b_weight_vec = vdupq_n_f32(b_weight);
  float32x4_t scale_vec = vdupq_n_f32(scale);

  int out_width = width / 2;

  for (int y = start_y; y < end_y; y += 2) {
    const uint16_t* row0 = bayer + y * width;
    const uint16_t* row1 = bayer + (y + 1) * width;
    uint8_t* out_row = grayscale + (y / 2) * out_width;

    int x = 0;
    // Process 8 output pixels at a time
    for (; x + 15 < width; x += 16) {
      uint16x8_t r0_low = vld1q_u16(row0 + x);
      uint16x8_t r0_high = vld1q_u16(row0 + x + 8);
      uint16x8_t r1_low = vld1q_u16(row1 + x);
      uint16x8_t r1_high = vld1q_u16(row1 + x + 8);

      uint16x8x2_t r0_deint = vuzpq_u16(r0_low, r0_high);
      uint16x8x2_t r1_deint = vuzpq_u16(r1_low, r1_high);

      // Convert to float and compute grayscale
      float32x4_t r_low =
          vcvtq_f32_u32(vmovl_u16(vget_low_u16(r0_deint.val[0])));
      float32x4_t g1_low =
          vcvtq_f32_u32(vmovl_u16(vget_low_u16(r0_deint.val[1])));
      float32x4_t g2_low =
          vcvtq_f32_u32(vmovl_u16(vget_low_u16(r1_deint.val[0])));
      float32x4_t b_low =
          vcvtq_f32_u32(vmovl_u16(vget_low_u16(r1_deint.val[1])));

      float32x4_t gray_low = vmulq_f32(r_low, r_weight_vec);
      gray_low = vmlaq_f32(gray_low, vaddq_f32(g1_low, g2_low), g_weight_vec);
      gray_low = vmlaq_f32(gray_low, b_low, b_weight_vec);
      gray_low = vmulq_f32(gray_low, scale_vec);

      float32x4_t r_high =
          vcvtq_f32_u32(vmovl_u16(vget_high_u16(r0_deint.val[0])));
      float32x4_t g1_high =
          vcvtq_f32_u32(vmovl_u16(vget_high_u16(r0_deint.val[1])));
      float32x4_t g2_high =
          vcvtq_f32_u32(vmovl_u16(vget_high_u16(r1_deint.val[0])));
      float32x4_t b_high =
          vcvtq_f32_u32(vmovl_u16(vget_high_u16(r1_deint.val[1])));

      float32x4_t gray_high = vmulq_f32(r_high, r_weight_vec);
      gray_high =
          vmlaq_f32(gray_high, vaddq_f32(g1_high, g2_high), g_weight_vec);
      gray_high = vmlaq_f32(gray_high, b_high, b_weight_vec);
      gray_high = vmulq_f32(gray_high, scale_vec);

      // Convert to uint8
      uint32x4_t gray_low_u32 = vcvtq_u32_f32(gray_low);
      uint32x4_t gray_high_u32 = vcvtq_u32_f32(gray_high);
      uint16x4_t gray_low_u16 = vqmovn_u32(gray_low_u32);
      uint16x4_t gray_high_u16 = vqmovn_u32(gray_high_u32);
      uint16x8_t gray_u16 = vcombine_u16(gray_low_u16, gray_high_u16);
      uint8x8_t gray_u8 = vqmovn_u16(gray_u16);

      vst1_u8(out_row + x / 2, gray_u8);
    }

    // Handle remaining pixels
    for (; x < width - 1; x += 2) {
      uint16_t r = row0[x];
      uint16_t g1 = row0[x + 1];
      uint16_t g2 = row1[x];
      uint16_t b = row1[x + 1];

      float g_avg = (g1 + g2) * 0.5f;
      float gray = r_weight * r + g_weight * g_avg + b_weight * b;
      gray *= scale;
      out_row[x / 2] =
          static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, gray)));
    }
  }
}

// Full resolution demosaic
void demosaic_full_res_fast(const uint16_t* bayer, uint8_t* grayscale,
                            int width, int height, int start_y, int end_y) {
  const float r_weight = 0.299f;
  const float g_weight = 0.587f;
  const float b_weight = 0.114f;
  const float scale = 255.0f / 1023.0f;

  for (int y = start_y; y < end_y; y++) {
    const uint16_t* curr_row = bayer + y * width;
    const uint16_t* prev_row = (y > 0) ? bayer + (y - 1) * width : curr_row;
    const uint16_t* next_row =
        (y < height - 1) ? bayer + (y + 1) * width : curr_row;
    uint8_t* out_row = grayscale + y * width;

    for (int x = 0; x < width; x++) {
      float r, g, b;

      bool is_even_row = (y % 2 == 0);
      bool is_even_col = (x % 2 == 0);

      if (is_even_row && is_even_col) {
        // Red pixel
        r = curr_row[x];

        float g_sum = 0;
        int g_count = 0;
        if (x > 0) {
          g_sum += curr_row[x - 1];
          g_count++;
        }
        if (x < width - 1) {
          g_sum += curr_row[x + 1];
          g_count++;
        }
        if (y > 0) {
          g_sum += prev_row[x];
          g_count++;
        }
        if (y < height - 1) {
          g_sum += next_row[x];
          g_count++;
        }
        g = g_count > 0 ? g_sum / g_count : 0;

        float b_sum = 0;
        int b_count = 0;
        if (x > 0 && y > 0) {
          b_sum += prev_row[x - 1];
          b_count++;
        }
        if (x < width - 1 && y > 0) {
          b_sum += prev_row[x + 1];
          b_count++;
        }
        if (x > 0 && y < height - 1) {
          b_sum += next_row[x - 1];
          b_count++;
        }
        if (x < width - 1 && y < height - 1) {
          b_sum += next_row[x + 1];
          b_count++;
        }
        b = b_count > 0 ? b_sum / b_count : 0;
      } else if (is_even_row && !is_even_col) {
        // Green pixel (R row)
        g = curr_row[x];

        float r_sum = 0;
        int r_count = 0;
        if (x > 0) {
          r_sum += curr_row[x - 1];
          r_count++;
        }
        if (x < width - 1) {
          r_sum += curr_row[x + 1];
          r_count++;
        }
        r = r_count > 0 ? r_sum / r_count : 0;

        float b_sum = 0;
        int b_count = 0;
        if (y > 0) {
          b_sum += prev_row[x];
          b_count++;
        }
        if (y < height - 1) {
          b_sum += next_row[x];
          b_count++;
        }
        b = b_count > 0 ? b_sum / b_count : 0;
      } else if (!is_even_row && is_even_col) {
        // Green pixel (B row)
        g = curr_row[x];

        float r_sum = 0;
        int r_count = 0;
        if (y > 0) {
          r_sum += prev_row[x];
          r_count++;
        }
        if (y < height - 1) {
          r_sum += next_row[x];
          r_count++;
        }
        r = r_count > 0 ? r_sum / r_count : 0;

        float b_sum = 0;
        int b_count = 0;
        if (x > 0) {
          b_sum += curr_row[x - 1];
          b_count++;
        }
        if (x < width - 1) {
          b_sum += curr_row[x + 1];
          b_count++;
        }
        b = b_count > 0 ? b_sum / b_count : 0;
      } else {
        // Blue pixel
        b = curr_row[x];

        float g_sum = 0;
        int g_count = 0;
        if (x > 0) {
          g_sum += curr_row[x - 1];
          g_count++;
        }
        if (x < width - 1) {
          g_sum += curr_row[x + 1];
          g_count++;
        }
        if (y > 0) {
          g_sum += prev_row[x];
          g_count++;
        }
        if (y < height - 1) {
          g_sum += next_row[x];
          g_count++;
        }
        g = g_count > 0 ? g_sum / g_count : 0;

        float r_sum = 0;
        int r_count = 0;
        if (x > 0 && y > 0) {
          r_sum += prev_row[x - 1];
          r_count++;
        }
        if (x < width - 1 && y > 0) {
          r_sum += prev_row[x + 1];
          r_count++;
        }
        if (x > 0 && y < height - 1) {
          r_sum += next_row[x - 1];
          r_count++;
        }
        if (x < width - 1 && y < height - 1) {
          r_sum += next_row[x + 1];
          r_count++;
        }
        r = r_count > 0 ? r_sum / r_count : 0;
      }

      float gray = (r_weight * r + g_weight * g + b_weight * b) * scale;
      out_row[x] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, gray)));
    }
  }
}

// Multi-threaded wrapper
void demosaic_threaded(const uint16_t* bayer, uint8_t* grayscale, int width,
                       int height, bool full_res, int num_threads) {
  std::vector<std::thread> threads;
  int rows_per_thread = height / num_threads;

  for (int i = 0; i < num_threads; i++) {
    int start_y = i * rows_per_thread;
    int end_y = (i == num_threads - 1) ? height : (i + 1) * rows_per_thread;

    if (!full_res && start_y % 2 != 0) {
      start_y--;
    }

    threads.emplace_back([=]() {
      if (full_res) {
        demosaic_full_res_fast(bayer, grayscale, width, height, start_y, end_y);
      } else {
        demosaic_half_res_neon(bayer, grayscale, width, height, start_y, end_y);
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }
}
