#include <gtest/gtest.h>

#include "cpp_helper_libs/math/arithmetic.hpp"

TEST(ProjectSmokeSharedTest, MathSharedLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(3, 4), 7);
}
