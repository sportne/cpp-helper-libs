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
  return contains_policy(point, true, false);
}

bool SphericalCircle::contains_exclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, false, false);
}

bool SphericalCircle::contains_inclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, true, true);
}

bool SphericalCircle::contains_exclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, false, true);
}

bool SphericalCircle::boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, true, false);
}

bool SphericalCircle::boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, false, false);
}

bool SphericalCircle::boundary_intersects_inclusive_exact(
    const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, true, true);
}

bool SphericalCircle::boundary_intersects_exclusive_exact(
    const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, false, true);
}

bool SphericalCircle::contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                      const bool inclusive, const bool exact) const noexcept {
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
