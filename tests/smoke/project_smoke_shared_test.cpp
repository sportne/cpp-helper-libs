#include <gtest/gtest.h>

#include "cpp_helper_libs/math/arithmetic.hpp"
#include "cpp_helper_libs/quantities/quantities.hpp"

TEST(ProjectSmokeSharedTest, MathSharedLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(3, 4), 7);
}

TEST(ProjectSmokeSharedTest, QuantitiesSharedLibraryLinksAndExecutes) {
  const auto speed = cpp_helper_libs::quantities::Speed::kilometers_per_hour(72.0);
  const auto angle = cpp_helper_libs::quantities::Angle::degrees(180.0);

  EXPECT_EQ(speed, cpp_helper_libs::quantities::Speed::meters_per_second(20.0));
  EXPECT_NEAR(angle.in(cpp_helper_libs::quantities::Angle::Unit::Radian), 3.14159265358979323846,
              1e-12);
}
