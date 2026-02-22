// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"

#include <algorithm>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi in radians; minor arcs must stay strictly below this.
constexpr double kPi = std::numbers::pi_v<double>;
// Numerical cushion to reject almost-zero and almost-pi sweeps.
constexpr double kSweepTolerance = 1e-12;

} // namespace

std::optional<MinorArc>
MinorArc::from_start_and_sweep(const SphericalRay &start_ray,
                               const cpp_helper_libs::quantities::Angle sweep) noexcept {
  const double sweep_radians = sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (sweep_radians <= kSweepTolerance || sweep_radians >= (kPi - kSweepTolerance)) {
    return std::nullopt;
  }

  return MinorArc(start_ray, sweep);
}

std::optional<MinorArc>
MinorArc::from_endpoints(const cpp_helper_libs::linear_algebra::UnitVector3 &start,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &end) noexcept {
  const double sweep_radians = start.central_angle_radians(end);
  if (sweep_radians <= kSweepTolerance || sweep_radians >= (kPi - kSweepTolerance)) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> normal =
      cpp_helper_libs::linear_algebra::unit_normal(start.as_vector(), end.as_vector());
  if (!normal.has_value()) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> tangent =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(normal->cross(start));
  if (!tangent.has_value()) {
    return std::nullopt;
  }

  const std::optional<SphericalRay> start_ray_value =
      SphericalRay::from_radial_and_tangent(start, tangent.value());
  if (!start_ray_value.has_value()) {
    return std::nullopt;
  }

  return MinorArc(start_ray_value.value(),
                  cpp_helper_libs::quantities::Angle::radians(sweep_radians));
}

cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::start_radial() const noexcept {
  return start_ray_.radial();
}

cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::end_radial() const noexcept {
  return end_ray().radial();
}

SphericalRay MinorArc::start_ray() const noexcept { return start_ray_; }

SphericalRay MinorArc::end_ray() const noexcept { return start_ray_.project_forward(sweep_); }

cpp_helper_libs::quantities::Angle MinorArc::length() const noexcept { return sweep_; }

cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::support_axis() const noexcept {
  return start_ray_.normal();
}

double MinorArc::support_constant() const noexcept { return 0.0; }

double MinorArc::signed_sweep_radians() const noexcept {
  return sweep_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
}

std::optional<CurveLocation>
MinorArc::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) const noexcept {
  return internal::locate_point_on_oriented_circle(
      support_axis(), support_constant(), start_radial(), signed_sweep_radians(), point, exact);
}

cpp_helper_libs::linear_algebra::UnitVector3
MinorArc::point_at_parameter(const double parameter) const noexcept {
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return internal::rotate_about_axis_tolerant(start_radial(), support_axis(),
                                              clamped * signed_sweep_radians());
}

bool MinorArc::is_zero_length_curve() const noexcept { return false; }

} // namespace cpp_helper_libs::spherical_geometry
