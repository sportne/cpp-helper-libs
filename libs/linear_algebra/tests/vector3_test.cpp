// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <numbers>
#include <optional>

#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"
#include "test_helpers.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::linear_algebra::unit_normal;
using cpp_helper_libs::linear_algebra::unit_tangent;
using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::linear_algebra::Vector3;
using cpp_helper_libs::linear_algebra::test::require_value;

TEST(Vector3Test, ConstructsAndComparesExactly) {
  const Vector3 lhs(1.0, 2.0, 3.0);
  const Vector3 rhs(1.0, 2.0, 3.0);
  const Vector3 near_value(1.0 + 1e-12, 2.0, 3.0);

  EXPECT_EQ(lhs.x(), 1.0);
  EXPECT_EQ(lhs.y(), 2.0);
  EXPECT_EQ(lhs.z(), 3.0);
  EXPECT_EQ(lhs, rhs);
  EXPECT_NE(lhs, near_value);
}

TEST(Vector3Test, SupportsArithmeticAndScalarMultiplication) {
  const Vector3 lhs(1.0, -2.0, 3.0);
  const Vector3 rhs(4.0, 5.0, -6.0);

  EXPECT_EQ(lhs + rhs, Vector3(5.0, 3.0, -3.0));
  EXPECT_EQ(lhs - rhs, Vector3(-3.0, -7.0, 9.0));
  EXPECT_EQ(-lhs, Vector3(-1.0, 2.0, -3.0));
  EXPECT_EQ(lhs * 2.0, Vector3(2.0, -4.0, 6.0));
  EXPECT_EQ(2.0 * lhs, Vector3(2.0, -4.0, 6.0));
}

TEST(Vector3Test, ComputesDotAndCrossProducts) {
  const Vector3 x_axis(1.0, 0.0, 0.0);
  const Vector3 y_axis(0.0, 1.0, 0.0);
  const Vector3 z_axis(0.0, 0.0, 1.0);

  EXPECT_DOUBLE_EQ(x_axis.dot(y_axis), 0.0);
  EXPECT_DOUBLE_EQ(x_axis.dot(x_axis), 1.0);
  EXPECT_EQ(x_axis.cross(y_axis), z_axis);
  EXPECT_EQ(y_axis.cross(x_axis), -z_axis);
}

TEST(Vector3Test, ComputesMagnitudeAndDistance) {
  const Vector3 value(3.0, 4.0, 12.0);
  const Vector3 origin(0.0, 0.0, 0.0);
  const Vector3 other(4.0, 6.0, 3.0);

  EXPECT_DOUBLE_EQ(value.squared_magnitude(), 169.0);
  EXPECT_DOUBLE_EQ(value.magnitude(), 13.0);
  EXPECT_DOUBLE_EQ(origin.euclidean_distance_to(other), 7.810249675906654);
}

TEST(Vector3Test, NormalizesAndHandlesZeroVector) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 normalized = require_value(Vector3(3.0, 0.0, 4.0).normalized());

  EXPECT_NEAR(normalized.x(), 0.6, kTolerance);
  EXPECT_NEAR(normalized.y(), 0.0, kTolerance);
  EXPECT_NEAR(normalized.z(), 0.8, kTolerance);
  EXPECT_FALSE(Vector3(0.0, 0.0, 0.0).normalized().has_value());
}

TEST(Vector3CentralAngleTest, ReturnsExpectedAnglesAndHandlesDegenerateInputs) {
  constexpr double kTolerance = 1e-12;
  constexpr double kPi = std::numbers::pi_v<double>;

  const Vector3 x_axis(1.0, 0.0, 0.0);
  const Vector3 y_axis(0.0, 1.0, 0.0);
  const Vector3 negative_x(-2.0, 0.0, 0.0);

  const std::optional<double> parallel_angle = x_axis.central_angle_radians(Vector3(2.0, 0.0, 0.0));
  const std::optional<double> orthogonal_angle = x_axis.central_angle_radians(y_axis);
  const std::optional<double> opposite_angle = x_axis.central_angle_radians(negative_x);

  ASSERT_TRUE(parallel_angle.has_value());
  ASSERT_TRUE(orthogonal_angle.has_value());
  ASSERT_TRUE(opposite_angle.has_value());

  EXPECT_NEAR(require_value(parallel_angle), 0.0, kTolerance);
  EXPECT_NEAR(require_value(orthogonal_angle), kPi / 2.0, kTolerance);
  EXPECT_NEAR(require_value(opposite_angle), kPi, kTolerance);

  const cpp_helper_libs::quantities::Angle as_angle = require_value(x_axis.central_angle(y_axis));
  EXPECT_NEAR(as_angle.in(cpp_helper_libs::quantities::Angle::Unit::Radian), kPi / 2.0, kTolerance);

  EXPECT_FALSE(x_axis.central_angle_radians(Vector3(0.0, 0.0, 0.0)).has_value());
  EXPECT_FALSE(Vector3(0.0, 0.0, 0.0).central_angle(y_axis).has_value());
}

TEST(UnitDirectionHelpersTest, ComputesUnitTangentAndUnitNormal) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 tangent = require_value(unit_tangent(Vector3(0.0, 3.0, 4.0)));
  EXPECT_NEAR(tangent.x(), 0.0, kTolerance);
  EXPECT_NEAR(tangent.y(), 0.6, kTolerance);
  EXPECT_NEAR(tangent.z(), 0.8, kTolerance);

  const UnitVector3 normal =
      require_value(unit_normal(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0)));
  EXPECT_NEAR(normal.x(), 0.0, kTolerance);
  EXPECT_NEAR(normal.y(), 0.0, kTolerance);
  EXPECT_NEAR(normal.z(), 1.0, kTolerance);
}

TEST(UnitDirectionHelpersTest, ReturnsNulloptForDegenerateInputs) {
  EXPECT_FALSE(unit_tangent(Vector3(0.0, 0.0, 0.0)).has_value());
  EXPECT_FALSE(unit_normal(Vector3(1.0, 0.0, 0.0), Vector3(2.0, 0.0, 0.0)).has_value());
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
