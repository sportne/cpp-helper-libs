#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/major_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"
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
using cpp_helper_libs::spherical_geometry::CurveIntersectionKind;
using cpp_helper_libs::spherical_geometry::MajorArc;
using cpp_helper_libs::spherical_geometry::MinorArc;
using cpp_helper_libs::spherical_geometry::SmallArc;
using cpp_helper_libs::spherical_geometry::SphericalCurve;
using cpp_helper_libs::spherical_geometry::SphericalRay;
using cpp_helper_libs::spherical_geometry::TurnDirection;
using cpp_helper_libs::spherical_geometry::ZeroLengthCurve;

TEST(SphericalCurveTest, ComputesArcLengths) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const MinorArc minor = require_value(MinorArc::from_endpoints(x_axis, y_axis));
  const SphericalRay start_ray =
      require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis));
  const MajorArc major =
      require_value(MajorArc::from_start_and_sweep(start_ray, Angle::degrees(270.0)));

  const UnitVector3 small_start =
      require_value(UnitVector3::from_components(std::sqrt(0.5), 0.0, std::sqrt(0.5)));
  const UnitVector3 small_tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay small_start_ray =
      require_value(SphericalRay::from_radial_and_tangent(small_start, small_tangent));
  const SmallArc small = require_value(SmallArc::from_center_start_direction_and_sweep(
      z_axis, Angle::degrees(45.0), small_start_ray, TurnDirection::CounterClockwise,
      Angle::degrees(90.0)));

  EXPECT_NEAR(minor.length().in(Angle::Unit::Radian), std::numbers::pi_v<double> / 2.0, kTolerance);
  EXPECT_NEAR(major.length().in(Angle::Unit::Radian), 3.0 * std::numbers::pi_v<double> / 2.0,
              kTolerance);
  EXPECT_NEAR(small.length().in(Angle::Unit::Radian),
              std::sin(std::numbers::pi_v<double> / 4.0) * (std::numbers::pi_v<double> / 2.0),
              kTolerance);
}

TEST(SphericalCurveTest, DetectsPointAndOverlapIntersections) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const MinorArc minor = require_value(MinorArc::from_endpoints(x_axis, y_axis));

  const SphericalRay major_start =
      require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis));
  const MajorArc major =
      require_value(MajorArc::from_start_and_sweep(major_start, Angle::degrees(270.0)));

  const UnitVector3 midpoint =
      require_value(UnitVector3::from_components(std::sqrt(0.5), std::sqrt(0.5), 0.0));
  const ZeroLengthCurve midpoint_curve = require_value(ZeroLengthCurve::at_radial(midpoint));

  const std::vector<cpp_helper_libs::spherical_geometry::CurveIntersection> overlap_hits =
      minor.intersections_with(major);
  ASSERT_FALSE(overlap_hits.empty());
  const bool saw_overlap =
      std::any_of(overlap_hits.begin(), overlap_hits.end(),
                  [](const cpp_helper_libs::spherical_geometry::CurveIntersection &hit) {
                    return hit.kind() == CurveIntersectionKind::OverlapSegment;
                  });
  EXPECT_TRUE(saw_overlap);

  const std::vector<cpp_helper_libs::spherical_geometry::CurveIntersection> point_hits =
      minor.intersections_with(midpoint_curve);
  ASSERT_EQ(point_hits.size(), 1U);
  EXPECT_EQ(point_hits.front().kind(), CurveIntersectionKind::EndpointTouch);

  const UnitVector3 equator_start = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const SphericalRay equator_start_ray =
      require_value(SphericalRay::from_radial_and_tangent(equator_start, y_axis));
  const SmallArc equator_quadrant = require_value(SmallArc::from_center_start_direction_and_sweep(
      z_axis, Angle::degrees(90.0), equator_start_ray, TurnDirection::CounterClockwise,
      Angle::degrees(90.0)));

  const std::vector<cpp_helper_libs::spherical_geometry::CurveIntersection> small_minor_hits =
      equator_quadrant.intersections_with(minor);
  ASSERT_FALSE(small_minor_hits.empty());
}

TEST(SphericalCurveTest, SupportsAllCurvePairings) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));

  const MinorArc minor = require_value(MinorArc::from_endpoints(x_axis, y_axis));
  const MajorArc major = require_value(MajorArc::from_start_and_sweep(
      require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis)), Angle::degrees(225.0)));
  const SmallArc small = require_value(SmallArc::from_center_start_direction_and_sweep(
      z_axis, Angle::degrees(90.0),
      require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis)),
      TurnDirection::CounterClockwise, Angle::degrees(120.0)));
  const ZeroLengthCurve zero = require_value(ZeroLengthCurve::at_radial(x_axis));

  const std::vector<const SphericalCurve *> curves = {&minor, &major, &small, &zero};

  for (const SphericalCurve *left : curves) {
    for (const SphericalCurve *right : curves) {
      const auto forward_hits = left->intersections_with(*right);
      const auto reverse_hits = right->intersections_with(*left);
      EXPECT_EQ(forward_hits.size(), reverse_hits.size());
    }
  }
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
