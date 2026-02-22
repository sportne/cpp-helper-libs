#include <gtest/gtest.h>

#include "cpp_helper_libs/math/arithmetic.hpp"

TEST(ProjectSmokeTest, MathLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(1, 2), 3);
}
