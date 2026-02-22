#include <gtest/gtest.h>

#include "cpp_helper_libs/math/arithmetic.hpp"
#include "cpp_helper_libs/quantities/quantities.hpp"

TEST(ProjectSmokeTest, MathLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(1, 2), 3);
}

TEST(ProjectSmokeTest, QuantitiesLibraryLinksAndExecutes) {
  const auto length = cpp_helper_libs::quantities::Length::meters(2.0);
  const auto time = cpp_helper_libs::quantities::Time::seconds(4.0);

  EXPECT_EQ(length + cpp_helper_libs::quantities::Length::meters(3.0),
            cpp_helper_libs::quantities::Length::meters(5.0));
  EXPECT_DOUBLE_EQ(time.in(cpp_helper_libs::quantities::Time::Unit::Millisecond), 4000.0);
}
