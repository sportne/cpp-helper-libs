#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#endif
#define protected public
#include "cpp_helper_libs/spherical_geometry/zero_length_curve.hpp"
#undef protected
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

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
using cpp_helper_libs::spherical_geometry::MinorArc;
using cpp_helper_libs::spherical_geometry::SphericalRay;
using cpp_helper_libs::spherical_geometry::ZeroLengthCurve;

TEST(ZeroLengthCurveTest, BuildsFromRayAndExposesDegenerateGeometry) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay ray = require_value(SphericalRay::from_radial_and_tangent(x_axis, y_axis));
  const ZeroLengthCurve curve = ZeroLengthCurve::at(ray);

  EXPECT_EQ(curve.start_radial(), x_axis);
  EXPECT_EQ(curve.end_radial(), x_axis);
  EXPECT_EQ(curve.start_ray().radial(), x_axis);
  EXPECT_EQ(curve.end_ray().radial(), x_axis);
  EXPECT_NEAR(curve.length().in(Angle::Unit::Radian), 0.0, kTolerance);
  EXPECT_NEAR(curve.support_constant(), 0.0, kTolerance);
  EXPECT_EQ(curve.support_axis(), ray.normal());
  EXPECT_DOUBLE_EQ(curve.signed_sweep_radians(), 0.0);
  EXPECT_TRUE(curve.is_zero_length_curve());
}

TEST(ZeroLengthCurveTest, AtRadialLocatesOnlyTheRepresentedPoint) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const ZeroLengthCurve curve = require_value(ZeroLengthCurve::at_radial(x_axis));

  const auto tolerant_location = curve.locate_point(x_axis, false);
  const auto exact_location = curve.locate_point(x_axis, true);
  ASSERT_TRUE(tolerant_location.has_value());
  ASSERT_TRUE(exact_location.has_value());
  const auto tolerant_location_value = require_value(tolerant_location);
  EXPECT_TRUE(tolerant_location_value.at_start);
  EXPECT_TRUE(tolerant_location_value.at_end);
  EXPECT_DOUBLE_EQ(tolerant_location_value.parameter, 0.0);
  EXPECT_FALSE(curve.locate_point(y_axis, false).has_value());
  EXPECT_EQ(curve.point_at_parameter(-1.0), x_axis);
  EXPECT_EQ(curve.point_at_parameter(2.0), x_axis);
}

TEST(ZeroLengthCurveTest, IntersectsOtherCurvesAtSharedEndpoints) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const ZeroLengthCurve point_curve = require_value(ZeroLengthCurve::at_radial(x_axis));
  const MinorArc arc = require_value(MinorArc::from_endpoints(x_axis, y_axis));

  const auto intersections = point_curve.intersections_with(arc);
  ASSERT_EQ(intersections.size(), 1U);
  EXPECT_EQ(intersections.front().kind(),
            cpp_helper_libs::spherical_geometry::CurveIntersectionKind::EndpointTouch);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
