// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <numbers>

#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"
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

TEST(ProjectSmokeTest, LinearAlgebraLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const cpp_helper_libs::linear_algebra::Vector3 x_axis(1.0, 0.0, 0.0);
  const cpp_helper_libs::linear_algebra::Vector3 y_axis(0.0, 1.0, 0.0);
  const auto right_normal = cpp_helper_libs::linear_algebra::unit_normal(x_axis, y_axis);
  const auto angle = x_axis.central_angle_radians(y_axis);

  if (!right_normal.has_value() || !angle.has_value()) {
    FAIL() << "Expected both unit normal and central angle to be defined";
  }

  const cpp_helper_libs::linear_algebra::UnitVector3 right_normal_value = right_normal.value();
  const double angle_value = angle.value();
  EXPECT_EQ(right_normal_value.as_vector(),
            cpp_helper_libs::linear_algebra::Vector3(0.0, 0.0, 1.0));
  EXPECT_NEAR(angle_value, std::numbers::pi_v<double> / 2.0, kTolerance);
}

TEST(ProjectSmokeTest, Matrix3LibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const cpp_helper_libs::linear_algebra::Matrix3 matrix_value(4.0, 1.0, 2.0, 1.0, 5.0, 1.0, 2.0,
                                                              1.0, 3.0);
  const cpp_helper_libs::linear_algebra::Vector3 rhs(7.0, 8.0, 5.0);

  const auto solved = matrix_value.solve(rhs);
  if (!solved.has_value()) {
    FAIL() << "Expected Matrix3 solve() to succeed";
  }

  const cpp_helper_libs::linear_algebra::Vector3 solved_value = solved.value();
  EXPECT_NEAR(solved_value.x(), 1.2162162162162162, kTolerance);
  EXPECT_NEAR(matrix_value.determinant(), 37.0, kTolerance);
}
