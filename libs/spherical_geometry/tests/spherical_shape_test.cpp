#include <gtest/gtest.h>

#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_circle.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ellipse.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_polygon.hpp"
#include "cpp_helper_libs/spherical_geometry/zero_length_curve.hpp"

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
using cpp_helper_libs::spherical_geometry::Coordinate;
using cpp_helper_libs::spherical_geometry::MinorArc;
using cpp_helper_libs::spherical_geometry::SphericalCircle;
using cpp_helper_libs::spherical_geometry::SphericalEllipse;
using cpp_helper_libs::spherical_geometry::SphericalPolygon;
using cpp_helper_libs::spherical_geometry::ZeroLengthCurve;

TEST(SphericalShapeTest, PolygonContainsAndBoundaryQueries) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const SphericalPolygon polygon =
      require_value(SphericalPolygon::from_vertices({x_axis, y_axis, z_axis}));
  const double one_over_sqrt_three = std::numbers::inv_sqrt3;
  const UnitVector3 inside = require_value(
      UnitVector3::from_components(one_over_sqrt_three, one_over_sqrt_three, one_over_sqrt_three));

  EXPECT_TRUE(polygon.contains_inclusive(inside));
  EXPECT_TRUE(polygon.contains_exclusive(inside));
  EXPECT_TRUE(polygon.contains_inclusive(x_axis));
  EXPECT_FALSE(polygon.contains_exclusive(x_axis));

  const ZeroLengthCurve boundary_point = require_value(ZeroLengthCurve::at_radial(x_axis));
  EXPECT_TRUE(polygon.boundary_intersects_inclusive(boundary_point));
  EXPECT_FALSE(polygon.boundary_intersects_exclusive(boundary_point));
}

TEST(SphericalShapeTest, CircleContainsAndBoundaryQueries) {
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const SphericalCircle circle =
      require_value(SphericalCircle::from_center_and_radius(z_axis, Angle::degrees(45.0)));

  const UnitVector3 inside = Coordinate::degrees(60.0, 0.0).to_radial();
  const UnitVector3 outside = Coordinate::degrees(0.0, 0.0).to_radial();
  const UnitVector3 boundary = Coordinate::degrees(45.0, 0.0).to_radial();

  EXPECT_TRUE(circle.contains_inclusive(inside));
  EXPECT_TRUE(circle.contains_exclusive(inside));
  EXPECT_FALSE(circle.contains_inclusive(outside));
  EXPECT_TRUE(circle.contains_inclusive(boundary));
  EXPECT_FALSE(circle.contains_exclusive(boundary));

  const ZeroLengthCurve boundary_point = require_value(ZeroLengthCurve::at_radial(boundary));
  EXPECT_TRUE(circle.boundary_intersects_inclusive(boundary_point));
  EXPECT_FALSE(circle.boundary_intersects_exclusive(boundary_point));
}

TEST(SphericalShapeTest, EllipseConstructionAndContainment) {
  const UnitVector3 focus_one = Coordinate::degrees(20.0, -20.0).to_radial();
  const UnitVector3 focus_two = Coordinate::degrees(20.0, 20.0).to_radial();

  const SphericalEllipse ellipse = require_value(SphericalEllipse::from_foci_and_boundary_sum(
      focus_one, focus_two, Angle::degrees(120.0), 120U));

  EXPECT_TRUE(ellipse.contains_inclusive(focus_one));
  EXPECT_TRUE(ellipse.contains_exclusive(focus_one));

  const UnitVector3 south_pole = Coordinate::degrees(-90.0, 0.0).to_radial();
  EXPECT_FALSE(ellipse.contains_inclusive(south_pole));

  const ZeroLengthCurve focus_curve = require_value(ZeroLengthCurve::at_radial(focus_one));
  EXPECT_FALSE(ellipse.boundary_intersects_inclusive(focus_curve));
}

TEST(SphericalShapeTest, PolygonBoundaryDetectsArcCrossing) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const SphericalPolygon polygon =
      require_value(SphericalPolygon::from_vertices({x_axis, y_axis, z_axis}));

  const MinorArc boundary_edge = require_value(MinorArc::from_endpoints(x_axis, y_axis));
  EXPECT_TRUE(polygon.boundary_intersects_inclusive(boundary_edge));
  EXPECT_TRUE(polygon.boundary_intersects_exclusive(boundary_edge));
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
