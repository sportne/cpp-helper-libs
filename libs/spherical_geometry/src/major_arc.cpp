// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/major_arc.hpp"

#include <algorithm>
#include <numbers>

#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi in radians; major arcs start at this sweep.
constexpr double kPi = std::numbers::pi_v<double>;
// Full turn in radians; major arcs must stay below this.
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
// Numerical cushion around valid sweep bounds.
constexpr double kSweepTolerance = 1e-12;

} // namespace

std::optional<MajorArc>
MajorArc::from_start_and_sweep(const SphericalRay &start_ray,
                               const cpp_helper_libs::quantities::Angle sweep) noexcept {
  const double sweep_radians = sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (sweep_radians < (kPi - kSweepTolerance) || sweep_radians >= (kTwoPi - kSweepTolerance)) {
    return std::nullopt;
  }

  return MajorArc(start_ray, sweep);
}

cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::start_radial() const noexcept {
  return start_ray_.radial();
}

cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::end_radial() const noexcept {
  return end_ray().radial();
}

SphericalRay MajorArc::start_ray() const noexcept { return start_ray_; }

SphericalRay MajorArc::end_ray() const noexcept { return start_ray_.project_forward(sweep_); }

cpp_helper_libs::quantities::Angle MajorArc::length() const noexcept { return sweep_; }

cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::support_axis() const noexcept {
  return start_ray_.normal();
}

double MajorArc::support_constant() const noexcept { return 0.0; }

double MajorArc::signed_sweep_radians() const noexcept {
  return sweep_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
}

std::optional<CurveLocation>
MajorArc::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) const noexcept {
  return internal::locate_point_on_oriented_circle(
      support_axis(), support_constant(), start_radial(), signed_sweep_radians(), point, exact);
}

cpp_helper_libs::linear_algebra::UnitVector3
MajorArc::point_at_parameter(const double parameter) const noexcept {
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return internal::rotate_about_axis_tolerant(start_radial(), support_axis(),
                                              clamped * signed_sweep_radians());
}

bool MajorArc::is_zero_length_curve() const noexcept { return false; }

} // namespace cpp_helper_libs::spherical_geometry
