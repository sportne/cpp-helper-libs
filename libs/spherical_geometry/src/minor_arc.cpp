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
  // Algorithm:
  // - Convert sweep to radians and require it in (0, pi).
  // - Apply `kSweepTolerance` to reject near-degenerate numeric edge cases.
  // Assumptions:
  // - Minor arcs must be strictly shorter than a half-turn.
  const double sweep_radians = sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (sweep_radians <= kSweepTolerance || sweep_radians >= (kPi - kSweepTolerance)) {
    return std::nullopt;
  }

  return MinorArc(start_ray, sweep);
}

std::optional<MinorArc>
MinorArc::from_endpoints(const cpp_helper_libs::linear_algebra::UnitVector3 &start,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &end) noexcept {
  // Algorithm:
  // - Compute great-circle distance between endpoints and enforce minor-arc bounds.
  // - Build great-circle normal from endpoint cross product.
  // - Derive start tangent as normal x start and construct start ray.
  // - Return the arc with computed sweep.
  // Assumptions:
  // - Endpoints are distinct and not antipodal for a unique minor arc.
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

// Algorithm:
// - Return the start ray radial direction.
// Assumptions:
// - Stored start frame remains valid for the life of the object.
cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::start_radial() const noexcept {
  return start_ray_.radial();
}

// Algorithm:
// - Evaluate end ray and return its radial direction.
// Assumptions:
// - End ray is computed by sweeping on the same great-circle support.
cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::end_radial() const noexcept {
  return end_ray().radial();
}

// Algorithm:
// - Return cached start frame.
// Assumptions:
// - MinorArc state is immutable.
SphericalRay MinorArc::start_ray() const noexcept { return start_ray_; }

// Algorithm:
// - Project start frame forward by full sweep.
// Assumptions:
// - Sweep sign matches forward tangent orientation.
SphericalRay MinorArc::end_ray() const noexcept { return start_ray_.project_forward(sweep_); }

// Algorithm:
// - Return stored sweep (equal to arc length on unit sphere).
// Assumptions:
// - Unit-sphere metric is used throughout this module.
cpp_helper_libs::quantities::Angle MinorArc::length() const noexcept { return sweep_; }

// Algorithm:
// - Return the normal of the underlying great-circle plane.
// Assumptions:
// - Great-circle support plane is fixed by start ray orientation.
cpp_helper_libs::linear_algebra::UnitVector3 MinorArc::support_axis() const noexcept {
  return start_ray_.normal();
}

// Algorithm:
// - Return zero for great-circle support equation axis · point = 0.
// Assumptions:
// - MinorArc is always on a great circle.
double MinorArc::support_constant() const noexcept { return 0.0; }

// Algorithm:
// - Convert stored sweep to radians.
// Assumptions:
// - This curve is parameterized in forward direction only.
double MinorArc::signed_sweep_radians() const noexcept {
  return sweep_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
}

std::optional<CurveLocation>
MinorArc::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) const noexcept {
  // Algorithm:
  // - Delegate point location to oriented-circle helper using this arc's support data.
  // Assumptions:
  // - Helper handles tolerance policy and endpoint classification.
  return internal::locate_point_on_oriented_circle(
      support_axis(), support_constant(), start_radial(), signed_sweep_radians(), point, exact);
}

cpp_helper_libs::linear_algebra::UnitVector3
MinorArc::point_at_parameter(const double parameter) const noexcept {
  // Algorithm:
  // - Clamp normalized parameter and rotate start radial accordingly.
  // Assumptions:
  // - Parameter 0 maps to start and 1 maps to end.
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return internal::rotate_about_axis_tolerant(start_radial(), support_axis(),
                                              clamped * signed_sweep_radians());
}

// Algorithm:
// - Return false because MinorArc never represents a degenerate point curve.
// Assumptions:
// - Constructor/factories reject zero-length sweep.
bool MinorArc::is_zero_length_curve() const noexcept { return false; }

} // namespace cpp_helper_libs::spherical_geometry
