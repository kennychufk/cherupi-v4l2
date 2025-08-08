#ifndef BAYER_CONVERTER_H
#define BAYER_CONVERTER_H

#include <cstdint>
#include <string>
#include <vector>

struct Config {
  std::string input_file;
  std::string output_file;
  int width = 1456;
  int height = 1088;
  int stride = 1824;
  bool full_res = false;
  int num_threads = 4;
  bool check_checkerboard = false;
  bool save_ppm = true;
};

class BayerConverter {
 public:
  BayerConverter(const Config& config);

  // Main processing function
  bool process();

  // Get the converted grayscale data
  const std::vector<uint8_t>& getGrayscaleData() const {
    return grayscale_data;
  }
  int getOutputWidth() const { return out_width; }
  int getOutputHeight() const { return out_height; }

 private:
  Config config;
  std::vector<uint8_t> packed_data;
  std::vector<uint16_t> bayer_data;
  std::vector<uint8_t> grayscale_data;
  int out_width;
  int out_height;

  // Processing functions
  void readInputFile();
  void unpack10BitData();
  void convertToGrayscale();
  void writePPM();
};

// Low-level functions for direct access if needed
void unpack_10bit_neon(const uint8_t* packed, uint16_t* unpacked, int width,
                       int height, int stride);

void demosaic_threaded(const uint16_t* bayer, uint8_t* grayscale, int width,
                       int height, bool full_res, int num_threads);

#endif  // BAYER_CONVERTER_H
