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
  // Algorithm:
  // - Convert sweep to radians and validate it lies in [pi, 2pi).
  // - Reject values too close to bounds using `kSweepTolerance`.
  // Assumptions:
  // - Major-arc semantics in this module include 180 degrees and exclude full-circle loops.
  const double sweep_radians = sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (sweep_radians < (kPi - kSweepTolerance) || sweep_radians >= (kTwoPi - kSweepTolerance)) {
    return std::nullopt;
  }

  return MajorArc(start_ray, sweep);
}

// Algorithm:
// - Return the radial component of the cached start ray.
// Assumptions:
// - `start_ray_` is an already validated spherical frame.
cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::start_radial() const noexcept {
  return start_ray_.radial();
}

// Algorithm:
// - Compute the end ray and return its radial component.
// Assumptions:
// - `end_ray()` evaluates forward motion on the same support great circle.
cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::end_radial() const noexcept {
  return end_ray().radial();
}

// Algorithm:
// - Return the cached start frame directly.
// Assumptions:
// - The object is immutable after construction.
SphericalRay MajorArc::start_ray() const noexcept { return start_ray_; }

// Algorithm:
// - Advance the start ray by the signed sweep along its great-circle direction.
// Assumptions:
// - `sweep_` is in the major-arc valid range checked at construction.
SphericalRay MajorArc::end_ray() const noexcept { return start_ray_.project_forward(sweep_); }

// Algorithm:
// - Return the stored central-angle sweep.
// Assumptions:
// - Arc length on the unit sphere equals central angle in radians.
cpp_helper_libs::quantities::Angle MajorArc::length() const noexcept { return sweep_; }

// Algorithm:
// - Use the start ray normal as the support-plane axis.
// Assumptions:
// - Great-circle supports always pass through the sphere center.
cpp_helper_libs::linear_algebra::UnitVector3 MajorArc::support_axis() const noexcept {
  return start_ray_.normal();
}

// Algorithm:
// - Return zero because great-circle planes satisfy axis · point = 0.
// Assumptions:
// - Major arcs are subsets of great circles.
double MajorArc::support_constant() const noexcept { return 0.0; }

// Algorithm:
// - Return sweep in radians without sign changes.
// Assumptions:
// - MajorArc is always parameterized in the forward direction of `start_ray_`.
double MajorArc::signed_sweep_radians() const noexcept {
  return sweep_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
}

std::optional<CurveLocation>
MajorArc::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) const noexcept {
  // Algorithm:
  // - Delegate to oriented-circle locator using this arc's support plane and sweep.
  // - The locator verifies both support-plane membership and parameter-range inclusion.
  // Assumptions:
  // - `support_axis`/`support_constant`/`signed_sweep_radians` satisfy curve contracts.
  return internal::locate_point_on_oriented_circle(
      support_axis(), support_constant(), start_radial(), signed_sweep_radians(), point, exact);
}

cpp_helper_libs::linear_algebra::UnitVector3
MajorArc::point_at_parameter(const double parameter) const noexcept {
  // Algorithm:
  // - Clamp parameter to [0,1] and rotate start radial by that fraction of total sweep.
  // Assumptions:
  // - Parameterization is linear in angular sweep.
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return internal::rotate_about_axis_tolerant(start_radial(), support_axis(),
                                              clamped * signed_sweep_radians());
}

// Algorithm:
// - Report false because this type always represents non-degenerate arc extents.
// Assumptions:
// - Factory rejects degenerate sweep values.
bool MajorArc::is_zero_length_curve() const noexcept { return false; }

} // namespace cpp_helper_libs::spherical_geometry
