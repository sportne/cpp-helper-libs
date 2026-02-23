// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/major_arc.hpp"

#include <numbers>

#include "great_circle_arc_common.hpp"

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

CPPHL_DEFINE_GREAT_CIRCLE_ARC_CORE_METHODS(MajorArc, start_ray_, sweep_)

} // namespace cpp_helper_libs::spherical_geometry
