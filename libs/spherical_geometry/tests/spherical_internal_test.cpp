#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "../src/spherical_internal.hpp"
#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return candidate.value();
}

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::spherical_geometry::internal::any_orthogonal_unit;
using cpp_helper_libs::spherical_geometry::internal::clamp_cosine;
using cpp_helper_libs::spherical_geometry::internal::exact_policy;
using cpp_helper_libs::spherical_geometry::internal::locate_point_on_oriented_circle;
using cpp_helper_libs::spherical_geometry::internal::make_ray_on_oriented_circle;
using cpp_helper_libs::spherical_geometry::internal::nearly_equal;
using cpp_helper_libs::spherical_geometry::internal::NumericPolicy;
using cpp_helper_libs::spherical_geometry::internal::rotate_about_axis_exact;
using cpp_helper_libs::spherical_geometry::internal::rotate_about_axis_tolerant;
using cpp_helper_libs::spherical_geometry::internal::same_radial;
using cpp_helper_libs::spherical_geometry::internal::select_policy;
using cpp_helper_libs::spherical_geometry::internal::small_arc_length_radians;
using cpp_helper_libs::spherical_geometry::internal::tolerant_policy;
using cpp_helper_libs::spherical_geometry::internal::wrap_zero_to_two_pi;

TEST(SphericalInternalTest, BuildsPolicyAndScalarHelpers) {
  const NumericPolicy tolerant = tolerant_policy();
  const NumericPolicy exact = exact_policy();
  EXPECT_GT(tolerant.support_epsilon, 0.0);
  EXPECT_GT(tolerant.parameter_epsilon, 0.0);
  EXPECT_GT(tolerant.radial_epsilon, 0.0);
  EXPECT_DOUBLE_EQ(exact.support_epsilon, 0.0);
  EXPECT_DOUBLE_EQ(exact.parameter_epsilon, 0.0);
  EXPECT_DOUBLE_EQ(exact.radial_epsilon, 0.0);
  EXPECT_DOUBLE_EQ(select_policy(true).support_epsilon, 0.0);
  EXPECT_GT(select_policy(false).support_epsilon, 0.0);

  EXPECT_TRUE(nearly_equal(1.0, 1.0 + 1e-10, 1e-9));
  EXPECT_FALSE(nearly_equal(1.0, 1.01, 1e-9));
  EXPECT_NEAR(wrap_zero_to_two_pi(-std::numbers::pi_v<double> / 2.0),
              1.5 * std::numbers::pi_v<double>, 1e-12);
  EXPECT_DOUBLE_EQ(clamp_cosine(2.0), 1.0);
  EXPECT_DOUBLE_EQ(clamp_cosine(-2.0), -1.0);
}

TEST(SphericalInternalTest, ComparesRadialsWithExactAndTolerantPolicies) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 tiny_offset =
      require_value(UnitVector3::from_components(std::cos(1e-6), std::sin(1e-6), 0.0));

  EXPECT_TRUE(same_radial(x_axis, x_axis, exact_policy()));
  EXPECT_FALSE(same_radial(x_axis, tiny_offset, exact_policy()));
  EXPECT_TRUE(same_radial(x_axis, tiny_offset, tolerant_policy()));
}

TEST(SphericalInternalTest, RotatesVectorsAndFindsOrthogonalBasis) {
  constexpr double kTolerance = 1e-12;
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const UnitVector3 rotated_exact =
      rotate_about_axis_exact(x_axis, z_axis, std::numbers::pi_v<double> / 2.0);
  const UnitVector3 rotated_tolerant =
      rotate_about_axis_tolerant(x_axis, z_axis, std::numbers::pi_v<double> / 2.0);

  EXPECT_NEAR(rotated_exact.x(), 0.0, kTolerance);
  EXPECT_NEAR(rotated_exact.y(), 1.0, kTolerance);
  EXPECT_NEAR(rotated_tolerant.x(), 0.0, kTolerance);
  EXPECT_NEAR(rotated_tolerant.y(), 1.0, kTolerance);

  const std::optional<UnitVector3> orthogonal = any_orthogonal_unit(z_axis);
  ASSERT_TRUE(orthogonal.has_value());
  const UnitVector3 orthogonal_value = require_value(orthogonal);
  EXPECT_NEAR(orthogonal_value.dot(z_axis), 0.0, kTolerance);
}

TEST(SphericalInternalTest, BuildsRayOnOrientedCircleInDegenerateAxisCase) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const auto ray = make_ray_on_oriented_circle(x_axis, x_axis, 1.0, false);

  EXPECT_EQ(ray.radial(), x_axis);
  EXPECT_NEAR(ray.radial().dot(ray.tangent()), 0.0, 1e-12);
}

TEST(SphericalInternalTest, LocatesPointsOnOrientedSweeps) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 negative_y = require_value(UnitVector3::from_components(0.0, -1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const auto quarter_turn = locate_point_on_oriented_circle(
      z_axis, 0.0, x_axis, std::numbers::pi_v<double> / 2.0, y_axis, false);
  ASSERT_TRUE(quarter_turn.has_value());
  const auto quarter_turn_value = require_value(quarter_turn);
  EXPECT_NEAR(quarter_turn_value.parameter, 1.0, kTolerance);
  EXPECT_TRUE(quarter_turn_value.at_end);

  const auto clockwise_quarter = locate_point_on_oriented_circle(
      z_axis, 0.0, x_axis, -std::numbers::pi_v<double> / 2.0, negative_y, false);
  ASSERT_TRUE(clockwise_quarter.has_value());
  const auto clockwise_quarter_value = require_value(clockwise_quarter);
  EXPECT_NEAR(clockwise_quarter_value.parameter, 1.0, kTolerance);
  EXPECT_TRUE(clockwise_quarter_value.at_end);

  const auto zero_sweep_at_start =
      locate_point_on_oriented_circle(z_axis, 0.0, x_axis, 0.0, x_axis, false);
  ASSERT_TRUE(zero_sweep_at_start.has_value());
  const auto zero_sweep_at_start_value = require_value(zero_sweep_at_start);
  EXPECT_TRUE(zero_sweep_at_start_value.at_start);
  EXPECT_TRUE(zero_sweep_at_start_value.at_end);

  EXPECT_FALSE(
      locate_point_on_oriented_circle(z_axis, 0.0, x_axis, 0.0, y_axis, false).has_value());
  EXPECT_FALSE(locate_point_on_oriented_circle(z_axis, 0.0, x_axis,
                                               std::numbers::pi_v<double> / 4.0, y_axis, true)
                   .has_value());
  EXPECT_FALSE(locate_point_on_oriented_circle(z_axis, 0.0, x_axis,
                                               std::numbers::pi_v<double> / 2.0, z_axis, false)
                   .has_value());
}

TEST(SphericalInternalTest, ComputesSmallArcLengthMagnitude) {
  EXPECT_NEAR(
      small_arc_length_radians(std::numbers::pi_v<double> / 6.0, -std::numbers::pi_v<double>),
      std::numbers::pi_v<double> / 2.0, 1e-12);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
