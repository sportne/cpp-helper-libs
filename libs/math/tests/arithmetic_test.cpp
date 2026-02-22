#include <gtest/gtest.h>

#include "cpp_helper_libs/math/arithmetic.hpp"

namespace {

TEST(ArithmeticAddTest, HandlesPositiveValues) {
  EXPECT_EQ(cpp_helper_libs::math::add(2, 3), 5);
}

TEST(ArithmeticAddTest, HandlesNegativeValues) {
  EXPECT_EQ(cpp_helper_libs::math::add(-2, -3), -5);
}

TEST(ArithmeticAddTest, HandlesZero) {
  EXPECT_EQ(cpp_helper_libs::math::add(0, 0), 0);
}

TEST(ArithmeticSubTest, HandlesPositiveValues) {
  EXPECT_EQ(cpp_helper_libs::math::sub(10, 4), 6);
}

TEST(ArithmeticSubTest, HandlesNegativeValues) {
  EXPECT_EQ(cpp_helper_libs::math::sub(-10, -4), -6);
}

TEST(ArithmeticSubTest, HandlesZero) {
  EXPECT_EQ(cpp_helper_libs::math::sub(0, 0), 0);
}

}  // namespace
