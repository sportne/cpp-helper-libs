// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi in radians; radius must stay in (0, pi) and sweep uses this for limits.
constexpr double kPi = std::numbers::pi_v<double>;
// Full turn in radians; small-arc sweep must be below this.
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
// Shared floating-point tolerance for geometric validation checks.
constexpr double kTolerance = 1e-10;

double turn_direction_sign(const TurnDirection direction) noexcept {
  return direction == TurnDirection::CounterClockwise ? 1.0 : -1.0;
}

} // namespace

std::optional<SmallArc> SmallArc::from_center_start_direction_and_sweep(
    const cpp_helper_libs::linear_algebra::UnitVector3 &center,
    const cpp_helper_libs::quantities::Angle radius, const SphericalRay &start_ray,
    const TurnDirection turn_direction, const cpp_helper_libs::quantities::Angle sweep) noexcept {
  const double radius_radians = radius.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double sweep_radians = sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  if (radius_radians <= kTolerance || radius_radians >= (kPi - kTolerance)) {
    return std::nullopt;
  }

  if (sweep_radians <= kTolerance || sweep_radians >= (kTwoPi - kTolerance)) {
    return std::nullopt;
  }

  const double radial_offset =
      std::fabs(center.central_angle_radians(start_ray.radial()) - radius_radians);
  if (radial_offset > kTolerance) {
    return std::nullopt;
  }

  const double sign = turn_direction_sign(turn_direction);
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> expected_tangent =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(center.cross(start_ray.radial()) *
                                                                sign);
  if (!expected_tangent.has_value()) {
    return std::nullopt;
  }

  if (expected_tangent->dot(start_ray.tangent()) < (1.0 - kTolerance)) {
    return std::nullopt;
  }

  return SmallArc(center, radius, start_ray, turn_direction, sweep);
}

cpp_helper_libs::linear_algebra::UnitVector3 SmallArc::start_radial() const noexcept {
  return start_ray_.radial();
}

cpp_helper_libs::linear_algebra::UnitVector3 SmallArc::end_radial() const noexcept {
  return point_at_parameter(1.0);
}

SphericalRay SmallArc::start_ray() const noexcept { return start_ray_; }

SphericalRay SmallArc::end_ray() const noexcept {
  const cpp_helper_libs::linear_algebra::UnitVector3 end_point = end_radial();
  const double sign = turn_direction_sign(turn_direction_);
  return internal::make_ray_on_oriented_circle(center_, end_point, sign, false);
}

cpp_helper_libs::quantities::Angle SmallArc::length() const noexcept {
  return cpp_helper_libs::quantities::Angle::radians(internal::small_arc_length_radians(
      radius_.in(cpp_helper_libs::quantities::Angle::Unit::Radian), signed_sweep_radians()));
}

cpp_helper_libs::linear_algebra::UnitVector3 SmallArc::support_axis() const noexcept {
  return center_;
}

double SmallArc::support_constant() const noexcept {
  return std::cos(radius_.in(cpp_helper_libs::quantities::Angle::Unit::Radian));
}

double SmallArc::signed_sweep_radians() const noexcept {
  const double sweep_radians = sweep_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  return turn_direction_sign(turn_direction_) * sweep_radians;
}

std::optional<CurveLocation>
SmallArc::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) const noexcept {
  return internal::locate_point_on_oriented_circle(
      support_axis(), support_constant(), start_radial(), signed_sweep_radians(), point, exact);
}

cpp_helper_libs::linear_algebra::UnitVector3
SmallArc::point_at_parameter(const double parameter) const noexcept {
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return internal::rotate_about_axis_tolerant(start_radial(), support_axis(),
                                              clamped * signed_sweep_radians());
}

bool SmallArc::is_zero_length_curve() const noexcept { return false; }

} // namespace cpp_helper_libs::spherical_geometry
