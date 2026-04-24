#include <gtest/gtest.h>

#include "types.hpp"

// Static Logger members — the test binary does not link main.cpp, so these
// must be defined here for the project sources to link.
LogLevel Logger::current_level = LogLevel::WARN;
std::mutex Logger::log_mutex;

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
