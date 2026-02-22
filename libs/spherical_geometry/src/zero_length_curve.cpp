// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/zero_length_curve.hpp"

#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
ZeroLengthCurve ZeroLengthCurve::at(const SphericalRay &ray) noexcept {
  // Algorithm:
  // - Wrap an existing ray directly as a degenerate point-curve.
  // Assumptions:
  // - Caller supplies a valid spherical frame.
  return ZeroLengthCurve(ray);
}

std::optional<ZeroLengthCurve>
ZeroLengthCurve::at_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept {
  // Algorithm:
  // - Pick any tangent orthogonal to radial.
  // - Build a spherical ray from radial+tangent.
  // - Wrap that ray as a zero-length curve.
  // Assumptions:
  // - Any tangent is acceptable because a point-curve has no swept direction.
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> tangent =
      internal::any_orthogonal_unit(radial);
  if (!tangent.has_value()) {
    return std::nullopt;
  }

  const std::optional<SphericalRay> ray =
      SphericalRay::from_radial_and_tangent(radial, tangent.value());
  if (!ray.has_value()) {
    return std::nullopt;
  }

  return ZeroLengthCurve(ray.value());
}

// Algorithm:
// - Return the single point radial represented by this degenerate curve.
// Assumptions:
// - `ray_` remains unchanged after construction.
cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::start_radial() const noexcept {
  return ray_.radial();
}

// Algorithm:
// - Return the same radial as `start_radial` because the curve has zero extent.
// Assumptions:
// - Start and end coincide for all zero-length curves.
cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::end_radial() const noexcept {
  return ray_.radial();
}

// Algorithm:
// - Return stored ray at the only point of the curve.
// Assumptions:
// - Orientation is arbitrary but fixed for this instance.
SphericalRay ZeroLengthCurve::start_ray() const noexcept { return ray_; }

// Algorithm:
// - Return the same ray as start because there is no traversal.
// Assumptions:
// - Endpoint orientation equality simplifies polymorphic consumers.
SphericalRay ZeroLengthCurve::end_ray() const noexcept { return ray_; }

// Algorithm:
// - Return a zero radian angle constant.
// Assumptions:
// - Curve length on unit sphere is exactly zero by definition.
cpp_helper_libs::quantities::Angle ZeroLengthCurve::length() const noexcept {
  return cpp_helper_libs::quantities::Angle::radians(0.0);
}

// Algorithm:
// - Use the stored normal as support axis.
// Assumptions:
// - Any plane containing the point is valid; this choice keeps support data coherent with ray_.
cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::support_axis() const noexcept {
  return ray_.normal();
}

// Algorithm:
// - Return axis · radial so the support equation includes the represented point.
// Assumptions:
// - Support data is used only for intersection plumbing, not to represent a unique circle.
double ZeroLengthCurve::support_constant() const noexcept {
  return ray_.normal().dot(ray_.radial());
}

// Algorithm:
// - Return zero sweep because there is no motion along the support.
// Assumptions:
// - Parameterization collapses to a single value at 0.
double ZeroLengthCurve::signed_sweep_radians() const noexcept { return 0.0; }

std::optional<CurveLocation>
ZeroLengthCurve::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                              const bool exact) const noexcept {
  // Algorithm:
  // - Compare candidate point to stored radial under selected numeric policy.
  // - If equal, report parameter 0 marked as both start and end.
  // Assumptions:
  // - Degenerate curves expose a single location with dual endpoint flags.
  const internal::NumericPolicy policy = internal::select_policy(exact);
  if (!internal::same_radial(ray_.radial(), point, policy)) {
    return std::nullopt;
  }

  return CurveLocation{.parameter = 0.0, .at_start = true, .at_end = true};
}

cpp_helper_libs::linear_algebra::UnitVector3
ZeroLengthCurve::point_at_parameter(const double /*parameter*/) const noexcept {
  // Algorithm:
  // - Ignore parameter and return the unique represented point.
  // Assumptions:
  // - Consumers may pass any parameter; output is constant.
  return ray_.radial();
}

// Algorithm:
// - Return true to activate zero-length special handling in intersection engine.
// Assumptions:
// - This class always models degenerate curves.
bool ZeroLengthCurve::is_zero_length_curve() const noexcept { return true; }

} // namespace cpp_helper_libs::spherical_geometry
