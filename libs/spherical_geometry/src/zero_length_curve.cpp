// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/zero_length_curve.hpp"

#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
ZeroLengthCurve ZeroLengthCurve::at(const SphericalRay &ray) noexcept {
  return ZeroLengthCurve(ray);
}

std::optional<ZeroLengthCurve>
ZeroLengthCurve::at_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept {
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

cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::start_radial() const noexcept {
  return ray_.radial();
}

cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::end_radial() const noexcept {
  return ray_.radial();
}

SphericalRay ZeroLengthCurve::start_ray() const noexcept { return ray_; }

SphericalRay ZeroLengthCurve::end_ray() const noexcept { return ray_; }

cpp_helper_libs::quantities::Angle ZeroLengthCurve::length() const noexcept {
  return cpp_helper_libs::quantities::Angle::radians(0.0);
}

cpp_helper_libs::linear_algebra::UnitVector3 ZeroLengthCurve::support_axis() const noexcept {
  return ray_.normal();
}

double ZeroLengthCurve::support_constant() const noexcept {
  return ray_.normal().dot(ray_.radial());
}

double ZeroLengthCurve::signed_sweep_radians() const noexcept { return 0.0; }

std::optional<CurveLocation>
ZeroLengthCurve::locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                              const bool exact) const noexcept {
  const internal::NumericPolicy policy = internal::select_policy(exact);
  if (!internal::same_radial(ray_.radial(), point, policy)) {
    return std::nullopt;
  }

  return CurveLocation{.parameter = 0.0, .at_start = true, .at_end = true};
}

cpp_helper_libs::linear_algebra::UnitVector3
ZeroLengthCurve::point_at_parameter(const double /*parameter*/) const noexcept {
  return ray_.radial();
}

bool ZeroLengthCurve::is_zero_length_curve() const noexcept { return true; }

} // namespace cpp_helper_libs::spherical_geometry
