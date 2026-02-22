// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

#include <cmath>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

// Allowed dot-product leakage when checking frame orthogonality.
constexpr double kFrameTolerance = 1e-10;

} // namespace

std::optional<SphericalRay>
SphericalRay::from_frame(const cpp_helper_libs::linear_algebra::UnitVector3 &radial,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &normal,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &tangent) noexcept {
  // Algorithm:
  // - Validate pairwise orthogonality of the three basis vectors.
  // - Recompute expected normal as radial x tangent.
  // - Accept only if recomputed normal agrees with the provided normal (same orientation).
  // Assumptions:
  // - Inputs are unit vectors (enforced by type), but may contain floating-point noise.
  if (std::fabs(radial.dot(normal)) > kFrameTolerance) {
    return std::nullopt;
  }
  if (std::fabs(radial.dot(tangent)) > kFrameTolerance) {
    return std::nullopt;
  }
  if (std::fabs(normal.dot(tangent)) > kFrameTolerance) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> expected_normal =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(radial.cross(tangent));
  if (!expected_normal.has_value()) {
    return std::nullopt;
  }

  if (!internal::same_radial(expected_normal.value(), normal, internal::tolerant_policy())) {
    return std::nullopt;
  }

  return SphericalRay(radial, normal, tangent);
}

std::optional<SphericalRay> SphericalRay::from_radial_and_tangent(
    const cpp_helper_libs::linear_algebra::UnitVector3 &radial,
    const cpp_helper_libs::linear_algebra::UnitVector3 &tangent) noexcept {
  // Algorithm:
  // - Ensure tangent is orthogonal to radial.
  // - Compute normal as radial x tangent to complete a right-handed frame.
  // - Build a ray from the validated frame.
  // Assumptions:
  // - Tangent direction encodes forward orientation along the curve.
  if (std::fabs(radial.dot(tangent)) > kFrameTolerance) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> computed_normal =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(radial.cross(tangent));
  if (!computed_normal.has_value()) {
    return std::nullopt;
  }

  return SphericalRay(radial, computed_normal.value(), tangent);
}

SphericalRay
SphericalRay::project_forward(const cpp_helper_libs::quantities::Angle arc_length) const noexcept {
  // Algorithm:
  // - Rotate both radial and tangent about the fixed normal by arc length.
  // - Reconstruct a ray from rotated vectors to re-validate frame consistency.
  // - Fall back to the original ray if reconstruction fails due to numerical issues.
  // Assumptions:
  // - Motion along a great-circle direction is modeled by rotation around `normal_`.
  const double radians = arc_length.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_radial =
      internal::rotate_about_axis_tolerant(radial_, normal_, radians);
  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_tangent =
      internal::rotate_about_axis_tolerant(tangent_, normal_, radians);

  const std::optional<SphericalRay> ray = from_radial_and_tangent(rotated_radial, rotated_tangent);
  if (ray.has_value()) {
    return ray.value();
  }

  return *this;
}

SphericalRay SphericalRay::project_forward_exact(
    const cpp_helper_libs::quantities::Angle arc_length) const noexcept {
  // Algorithm:
  // - Same as `project_forward`, but use exact-policy axis rotation helper.
  // Assumptions:
  // - Exact mode disables tolerance shortcuts but still protects against impossible frame
  //   reconstruction by returning `*this`.
  const double radians = arc_length.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_radial =
      internal::rotate_about_axis_exact(radial_, normal_, radians);
  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_tangent =
      internal::rotate_about_axis_exact(tangent_, normal_, radians);

  const std::optional<SphericalRay> ray = from_radial_and_tangent(rotated_radial, rotated_tangent);
  if (ray.has_value()) {
    return ray.value();
  }

  return *this;
}

SphericalRay
SphericalRay::rotate_about_radial(const cpp_helper_libs::quantities::Angle angle) const noexcept {
  // Algorithm:
  // - Keep radial fixed.
  // - Rotate tangent and normal about radial by the requested roll angle.
  // - Rebuild a validated frame from the rotated vectors.
  // Assumptions:
  // - This operation changes heading/orientation only, not spherical position.
  const double radians = angle.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_tangent =
      internal::rotate_about_axis_tolerant(tangent_, radial_, radians);
  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_normal =
      internal::rotate_about_axis_tolerant(normal_, radial_, radians);

  const std::optional<SphericalRay> ray = from_frame(radial_, rotated_normal, rotated_tangent);
  if (ray.has_value()) {
    return ray.value();
  }

  return *this;
}

SphericalRay SphericalRay::rotate_about_radial_exact(
    const cpp_helper_libs::quantities::Angle angle) const noexcept {
  // Algorithm:
  // - Exact-policy counterpart of `rotate_about_radial`.
  // Assumptions:
  // - Fallback-to-self behavior matches tolerant mode for API stability.
  const double radians = angle.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_tangent =
      internal::rotate_about_axis_exact(tangent_, radial_, radians);
  const cpp_helper_libs::linear_algebra::UnitVector3 rotated_normal =
      internal::rotate_about_axis_exact(normal_, radial_, radians);

  const std::optional<SphericalRay> ray = from_frame(radial_, rotated_normal, rotated_tangent);
  if (ray.has_value()) {
    return ray.value();
  }

  return *this;
}

} // namespace cpp_helper_libs::spherical_geometry
