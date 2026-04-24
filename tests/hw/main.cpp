#include <gtest/gtest.h>

#include "types.hpp"

LogLevel Logger::current_level = LogLevel::WARN;
std::mutex Logger::log_mutex;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
