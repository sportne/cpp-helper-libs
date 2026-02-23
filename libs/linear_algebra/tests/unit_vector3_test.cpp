// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <numbers>
#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "test_helpers.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::linear_algebra::Vector3;
using cpp_helper_libs::linear_algebra::test::require_value;

TEST(UnitVector3FactoryTest, ValidatesUnitLengthInvariant) {
  const std::optional<UnitVector3> valid = UnitVector3::from_components(1.0, 0.0, 0.0);
  const std::optional<UnitVector3> invalid = UnitVector3::from_components(1.0 + 1e-9, 0.0, 0.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::optional<UnitVector3> non_finite = UnitVector3::from_components(nan, 0.0, 0.0);

  EXPECT_TRUE(valid.has_value());
  EXPECT_FALSE(invalid.has_value());
  EXPECT_FALSE(non_finite.has_value());
}

TEST(UnitVector3FactoryTest, CreatesFromNonZeroVector) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 value = require_value(UnitVector3::from_vector(Vector3(3.0, 4.0, 0.0)));

  EXPECT_NEAR(value.x(), 0.6, kTolerance);
  EXPECT_NEAR(value.y(), 0.8, kTolerance);
  EXPECT_NEAR(value.z(), 0.0, kTolerance);
  EXPECT_FALSE(UnitVector3::from_vector(Vector3(0.0, 0.0, 0.0)).has_value());
}

TEST(UnitVector3Test, SupportsVectorOperations) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));

  EXPECT_EQ(x_axis.as_vector(), Vector3(1.0, 0.0, 0.0));
  EXPECT_EQ(x_axis.scaled_by(4.0), Vector3(4.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(x_axis.dot(y_axis), 0.0);
  EXPECT_EQ(x_axis.cross(y_axis), Vector3(0.0, 0.0, 1.0));
  EXPECT_EQ(-x_axis, require_value(UnitVector3::from_components(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(x_axis != y_axis);
}

TEST(UnitVector3CentralAngleTest, ReturnsExpectedAngles) {
  constexpr double kTolerance = 1e-12;
  constexpr double kPi = std::numbers::pi_v<double>;

  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 negative_x = require_value(UnitVector3::from_components(-1.0, 0.0, 0.0));

  EXPECT_NEAR(x_axis.central_angle_radians(x_axis), 0.0, kTolerance);
  EXPECT_NEAR(x_axis.central_angle_radians(y_axis), kPi / 2.0, kTolerance);
  EXPECT_NEAR(x_axis.central_angle_radians(negative_x), kPi, kTolerance);
  EXPECT_NEAR(x_axis.central_angle(y_axis).in(cpp_helper_libs::quantities::Angle::Unit::Radian),
              kPi / 2.0, kTolerance);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
