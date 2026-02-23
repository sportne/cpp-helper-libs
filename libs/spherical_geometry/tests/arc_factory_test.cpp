#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/major_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"
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
using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::spherical_geometry::MajorArc;
using cpp_helper_libs::spherical_geometry::MinorArc;
using cpp_helper_libs::spherical_geometry::SmallArc;
using cpp_helper_libs::spherical_geometry::SphericalRay;
using cpp_helper_libs::spherical_geometry::TurnDirection;

TEST(ArcFactoryTest, ValidatesMinorArcConstructionPaths) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 negative_x = require_value(UnitVector3::from_components(-1.0, 0.0, 0.0));

  const SphericalRay start_ray = require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis));

  EXPECT_FALSE(MinorArc::from_start_and_sweep(start_ray, Angle::degrees(0.0)).has_value());
  EXPECT_FALSE(MinorArc::from_start_and_sweep(start_ray, Angle::degrees(180.0)).has_value());

  const MinorArc arc = require_value(MinorArc::from_start_and_sweep(start_ray, Angle::degrees(45.0)));
  EXPECT_EQ(arc.start_ray().radial(), x_axis);

  EXPECT_FALSE(MinorArc::from_endpoints(x_axis, x_axis).has_value());
  EXPECT_FALSE(MinorArc::from_endpoints(x_axis, negative_x).has_value());
}

TEST(ArcFactoryTest, ValidatesMajorArcConstructionBounds) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay start_ray = require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis));

  EXPECT_FALSE(MajorArc::from_start_and_sweep(start_ray, Angle::degrees(90.0)).has_value());
  EXPECT_FALSE(MajorArc::from_start_and_sweep(start_ray, Angle::degrees(360.0)).has_value());

  const MajorArc arc = require_value(MajorArc::from_start_and_sweep(start_ray, Angle::degrees(180.0)));
  EXPECT_NEAR(arc.length().in(Angle::Unit::Radian), std::numbers::pi_v<double>, 1e-12);
  EXPECT_EQ(arc.start_ray().radial(), x_axis);
}

TEST(ArcFactoryTest, ValidatesSmallArcConstructionBounds) {
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));
  const UnitVector3 start_radial =
      require_value(UnitVector3::from_components(std::sqrt(0.5), 0.0, std::sqrt(0.5)));
  const UnitVector3 start_tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay start_ray =
      require_value(SphericalRay::from_radial_and_tangent(start_radial, start_tangent));

  EXPECT_FALSE(SmallArc::from_center_start_direction_and_sweep(
                   z_axis, Angle::degrees(0.0), start_ray, TurnDirection::CounterClockwise,
                   Angle::degrees(45.0))
                   .has_value());
  EXPECT_FALSE(SmallArc::from_center_start_direction_and_sweep(
                   z_axis, Angle::degrees(45.0), start_ray, TurnDirection::CounterClockwise,
                   Angle::degrees(0.0))
                   .has_value());

  const UnitVector3 wrong_start = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const SphericalRay wrong_start_ray =
      require_value(SphericalRay::from_radial_and_tangent(wrong_start, start_tangent));
  EXPECT_FALSE(SmallArc::from_center_start_direction_and_sweep(
                   z_axis, Angle::degrees(45.0), wrong_start_ray, TurnDirection::CounterClockwise,
                   Angle::degrees(45.0))
                   .has_value());

  const SmallArc valid_arc = require_value(SmallArc::from_center_start_direction_and_sweep(
      z_axis, Angle::degrees(45.0), start_ray, TurnDirection::CounterClockwise,
      Angle::degrees(90.0)));
  EXPECT_EQ(valid_arc.start_ray().radial(), start_radial);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
