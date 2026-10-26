// The server compiles with -ffast-math, and its plain form implies
// -ffinite-math-only: the compiler may then assume NaN never occurs, folding
// std::isnan / std::isfinite to constants. The server relies on NaN to mean
// "not reported" (LensPosition, a fixed-focus module's lens_position_limits,
// the stacker's gain and black level), and with that folding a fixed-focus
// IMX477 sent `"min":2.696539702293474e+308` — which no JSON parser accepts —
// instead of null. This binary is built with the server's CHERUPI_MATH_FLAGS,
// so these fail if the flags ever drop -fno-finite-math-only again.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>

#include "types.hpp"

namespace {

// A NaN the compiler cannot see through, as one read from libcamera is.
float runtimeNaN() {
  volatile float v = std::numeric_limits<float>::quiet_NaN();
  return v;
}

TEST(FloatSemanticsTest, NaNChecksAreNotFoldedAway) {
  const float nan = runtimeNaN();
  EXPECT_TRUE(std::isnan(nan));
  EXPECT_FALSE(std::isfinite(nan));
  EXPECT_NE(nan, nan);
}

TEST(FloatSemanticsTest, UnavailableLensLimitsSerialiseAsNull) {
  // Built as handleGetLensPositionLimits builds its reply.
  const float nan = runtimeNaN();
  const LensPositionLimits limits{nan, nan, nan};
  nlohmann::json response;
  response["min"] = limits.min;
  response["max"] = limits.max;
  response["default"] = limits.def;

  const std::string text = response.dump();
  EXPECT_EQ(text, R"({"default":null,"max":null,"min":null})");
  // And it reads back, which the garbage-number form did not.
  nlohmann::json back;
  EXPECT_NO_THROW(back = nlohmann::json::parse(text));
  EXPECT_TRUE(back["min"].is_null());
}

}  // namespace
