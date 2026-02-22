#include <gtest/gtest.h>

#include <numbers>

#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"
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

TEST(ProjectSmokeSharedTest, LinearAlgebraSharedLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const auto tangent = cpp_helper_libs::linear_algebra::unit_tangent(
      cpp_helper_libs::linear_algebra::Vector3(0.0, 3.0, 4.0));
  const auto x_axis = cpp_helper_libs::linear_algebra::UnitVector3::from_components(1.0, 0.0, 0.0);
  const auto y_axis = cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 1.0, 0.0);

  if (!tangent.has_value() || !x_axis.has_value() || !y_axis.has_value()) {
    FAIL() << "Expected tangent and basis unit vectors to be defined";
  }

  const cpp_helper_libs::linear_algebra::UnitVector3 tangent_value = tangent.value();
  const cpp_helper_libs::linear_algebra::UnitVector3 x_axis_value = x_axis.value();
  const cpp_helper_libs::linear_algebra::UnitVector3 y_axis_value = y_axis.value();
  EXPECT_NEAR(tangent_value.z(), 0.8, kTolerance);
  EXPECT_NEAR(x_axis_value.central_angle_radians(y_axis_value), std::numbers::pi_v<double> / 2.0,
              kTolerance);
}
