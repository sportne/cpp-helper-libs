// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_circle.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
class SphericalCurve;
} // namespace cpp_helper_libs::spherical_geometry

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi in radians; radius must stay in (0, pi).
constexpr double kPi = std::numbers::pi_v<double>;
// Tolerance used for containment and radius validation checks.
constexpr double kTolerance = 1e-10;

std::optional<std::vector<SmallArc>>
build_boundary_arcs(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                    const cpp_helper_libs::quantities::Angle radius) {
  // Algorithm:
  // - Pick one deterministic point on the circle by combining center and an orthogonal direction.
  // - Build a tangent at that point and create a start ray on the boundary.
  // - Split the full loop into two counter-clockwise pi sweeps (two SmallArc halves).
  // Assumptions:
  // - Representing the boundary as two half-arcs avoids requiring a full 2pi arc primitive.
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> orthogonal =
      internal::any_orthogonal_unit(center);
  if (!orthogonal.has_value()) {
    return std::nullopt;
  }

  const double radius_radians = radius.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const cpp_helper_libs::linear_algebra::Vector3 start_vector =
      center.scaled_by(std::cos(radius_radians)) + orthogonal->scaled_by(std::sin(radius_radians));
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> start_radial =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(start_vector);
  if (!start_radial.has_value()) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> start_tangent =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(center.cross(start_radial.value()));
  if (!start_tangent.has_value()) {
    return std::nullopt;
  }

  const std::optional<SphericalRay> start_ray =
      SphericalRay::from_radial_and_tangent(start_radial.value(), start_tangent.value());
  if (!start_ray.has_value()) {
    return std::nullopt;
  }

  const std::optional<SmallArc> first_half = SmallArc::from_center_start_direction_and_sweep(
      center, radius, start_ray.value(), TurnDirection::CounterClockwise,
      cpp_helper_libs::quantities::Angle::radians(kPi));
  if (!first_half.has_value()) {
    return std::nullopt;
  }

  const std::optional<SmallArc> second_half = SmallArc::from_center_start_direction_and_sweep(
      center, radius, first_half->end_ray(), TurnDirection::CounterClockwise,
      cpp_helper_libs::quantities::Angle::radians(kPi));
  if (!second_half.has_value()) {
    return std::nullopt;
  }

  return std::vector<SmallArc>{first_half.value(), second_half.value()};
}

} // namespace

std::optional<SphericalCircle>
SphericalCircle::from_center_and_radius(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                                        const cpp_helper_libs::quantities::Angle radius) noexcept {
  // Algorithm:
  // - Validate radius is strictly between 0 and pi.
  // - Build a pair of SmallArc boundary segments covering the full circle.
  // - Construct the shape from validated geometry.
  // Assumptions:
  // - Radius is interpreted as central angle on the unit sphere.
  const double radius_radians = radius.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (radius_radians <= kTolerance || radius_radians >= (kPi - kTolerance)) {
    return std::nullopt;
  }

  const std::optional<std::vector<SmallArc>> boundary_arcs = build_boundary_arcs(center, radius);
  if (!boundary_arcs.has_value()) {
    return std::nullopt;
  }

  return SphericalCircle(center, radius, boundary_arcs.value());
}

bool SphericalCircle::contains_inclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to shared containment policy with inclusive boundary semantics.
  // Assumptions:
  // - Tolerant comparison mode is desired for default API.
  return contains_policy(point, true, false);
}

bool SphericalCircle::contains_exclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to shared containment policy with strict interior semantics.
  // Assumptions:
  // - Boundary points are excluded in this mode.
  return contains_policy(point, false, false);
}

bool SphericalCircle::contains_inclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to shared containment policy with exact numeric comparisons.
  // Assumptions:
  // - No tolerance slack is applied in exact mode.
  return contains_policy(point, true, true);
}

bool SphericalCircle::contains_exclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to shared containment policy with exact + exclusive semantics.
  // Assumptions:
  // - Exact mode treats points at radius as outside for exclusive containment.
  return contains_policy(point, false, true);
}

bool SphericalCircle::boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to shared boundary-intersection policy with inclusive semantics.
  // Assumptions:
  // - Endpoint-only touches count as intersections in inclusive mode.
  return boundary_intersects_policy(curve, true, false);
}

bool SphericalCircle::boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to shared boundary-intersection policy with exclusive semantics.
  // Assumptions:
  // - Endpoint-only touches should be filtered out.
  return boundary_intersects_policy(curve, false, false);
}

bool SphericalCircle::boundary_intersects_inclusive_exact(
    const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to exact-policy boundary intersection with inclusive semantics.
  // Assumptions:
  // - Exact mode disables tolerance in the underlying curve intersections.
  return boundary_intersects_policy(curve, true, true);
}

bool SphericalCircle::boundary_intersects_exclusive_exact(
    const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to exact-policy boundary intersection with exclusive semantics.
  // Assumptions:
  // - Endpoint touches are ignored in exclusive mode.
  return boundary_intersects_policy(curve, false, true);
}

bool SphericalCircle::contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                      const bool inclusive, const bool exact) const noexcept {
  // Algorithm:
  // - Compute central-angle distance from center to point.
  // - Compare distance to radius with optional tolerance slack.
  // - Use <= for inclusive, < for exclusive.
  // Assumptions:
  // - Shape interior is the spherical cap bounded by the small circle.
  const double distance = center_.central_angle_radians(point);
  const double radius_value = radius_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double epsilon = exact ? 0.0 : kTolerance;

  if (inclusive) {
    return distance <= (radius_value + epsilon);
  }

  return distance < (radius_value - epsilon);
}

bool SphericalCircle::boundary_intersects_policy(const SphericalCurve &curve, const bool inclusive,
                                                 const bool exact) const noexcept {
  // Algorithm:
  // - Intersect the query curve with each boundary half-arc.
  // - Return true immediately for inclusive mode when any intersection exists.
  // - In exclusive mode, require at least one non-endpoint-touch intersection.
  // Assumptions:
  // - `boundary_arcs_` forms a complete closed boundary with no gaps.
  return std::any_of(
      boundary_arcs_.begin(), boundary_arcs_.end(), [&](const SmallArc &boundary_arc) {
        const std::vector<CurveIntersection> intersections =
            exact ? boundary_arc.intersections_with_exact(curve)
                  : boundary_arc.intersections_with(curve);

        if (intersections.empty()) {
          return false;
        }

        if (inclusive) {
          return true;
        }

        return std::any_of(intersections.begin(), intersections.end(),
                           [](const CurveIntersection &intersection) {
                             return intersection.kind() != CurveIntersectionKind::EndpointTouch;
                           });
      });
}

} // namespace cpp_helper_libs::spherical_geometry
