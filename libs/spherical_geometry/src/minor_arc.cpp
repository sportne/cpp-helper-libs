// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"

#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "great_circle_arc_common.hpp"

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

CPPHL_DEFINE_GREAT_CIRCLE_ARC_CORE_METHODS(MinorArc, start_ray_, sweep_)

} // namespace cpp_helper_libs::spherical_geometry
