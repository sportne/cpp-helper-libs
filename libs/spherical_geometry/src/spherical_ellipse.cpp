// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_ellipse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

namespace {

// pi in radians; used for polar-angle search bounds.
constexpr double kPi = std::numbers::pi_v<double>;
// Full turn in radians.
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
// Tolerance for "inside ellipse" sum-angle comparisons.
constexpr double kContainmentTolerance = 1e-10;
// Number of coarse samples while searching polar-angle roots per azimuth.
constexpr std::size_t kBoundarySearchResolution = 720U;
// Maximum bisection iterations for each root solve.
constexpr std::size_t kBisectIterationLimit = 64U;
// Convergence threshold for the bisection residual.
constexpr double kBisectResidualTolerance = 1e-12;
// Acceptable residual when no sign change is found but a close sample exists.
constexpr double kBoundaryApproximateResidualTolerance = 1e-6;
// Shared 0.5 constant for midpoint computations.
constexpr double kHalf = 0.5;
// Smallest supported segment count for a stable boundary approximation.
constexpr std::size_t kMinimumBoundarySegmentCount = 8U;

struct RootBracket final {
  // Lower polar-angle bound for a sign-changing interval.
  double lower;
  // Upper polar-angle bound for a sign-changing interval.
  double upper;
};

struct BracketResiduals final {
  // Residual at RootBracket::lower.
  double lower;
  // Residual at RootBracket::upper.
  double upper;
};

struct BoundaryDiscretization final {
  // Target sum of distances to the two foci for boundary points.
  double boundary_sum_radians;
  // Number of boundary samples/segments used in the polygonal approximation.
  std::size_t segment_count;
};

// Convert local spherical coordinates (polar angle + precomputed azimuth direction)
// into a unit radial on the sphere.
// Assumption: axis and azimuth_direction form a valid local frame direction.
cpp_helper_libs::linear_algebra::UnitVector3
point_on_basis_with_direction(const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                              const cpp_helper_libs::linear_algebra::Vector3 &azimuth_direction,
                              const double polar_angle) {
  const cpp_helper_libs::linear_algebra::Vector3 point =
      axis.scaled_by(std::cos(polar_angle)) + azimuth_direction * std::sin(polar_angle);

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> point_unit =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(point);
  if (point_unit.has_value()) {
    return point_unit.value();
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> unit_z =
      cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 0.0, 1.0);
  if (unit_z.has_value()) {
    return unit_z.value();
  }

  return axis;
}

cpp_helper_libs::linear_algebra::Vector3
azimuth_direction(const cpp_helper_libs::linear_algebra::UnitVector3 &basis_first,
                  const cpp_helper_libs::linear_algebra::UnitVector3 &basis_second,
                  const double azimuth) {
  return basis_first.scaled_by(std::cos(azimuth)) + basis_second.scaled_by(std::sin(azimuth));
}

// Residual function for the ellipse boundary equation:
// angle(focus_one, point) + angle(focus_two, point) - boundary_sum = 0.
double boundary_residual(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                         const double boundary_sum_radians) {
  return focus_one.central_angle_radians(point) + focus_two.central_angle_radians(point) -
         boundary_sum_radians;
}

double boundary_residual_for_polar(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                                   const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                                   const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                                   const cpp_helper_libs::linear_algebra::Vector3 &direction,
                                   const double polar_angle, const double boundary_sum_radians) {
  return boundary_residual(focus_one, focus_two,
                           point_on_basis_with_direction(axis, direction, polar_angle),
                           boundary_sum_radians);
}

// Solve one boundary root inside a bracket where the residual changes sign.
// Algorithm: bisection with fixed iteration cap and residual tolerance.
std::optional<double>
root_from_sign_change(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                      const cpp_helper_libs::linear_algebra::Vector3 &direction,
                      const double boundary_sum_radians, const RootBracket bracket,
                      const BracketResiduals bracket_residuals) {
  double lower = bracket.lower;
  double upper = bracket.upper;
  double residual_lower = bracket_residuals.lower;
  const double residual_upper = bracket_residuals.upper;

  if (residual_lower == 0.0) {
    return lower;
  }
  if (residual_upper == 0.0) {
    return upper;
  }

  if (residual_lower * residual_upper > 0.0) {
    return std::nullopt;
  }

  for (std::size_t iteration = 0U; iteration < kBisectIterationLimit; ++iteration) {
    const double midpoint = kHalf * (lower + upper);
    const double residual_midpoint = boundary_residual_for_polar(
        focus_one, focus_two, axis, direction, midpoint, boundary_sum_radians);

    if (std::fabs(residual_midpoint) < kBisectResidualTolerance) {
      return midpoint;
    }

    if (residual_midpoint * residual_lower < 0.0) {
      upper = midpoint;
    } else {
      lower = midpoint;
      residual_lower = residual_midpoint;
    }
  }

  return kHalf * (lower + upper);
}

// Fast local solve near previous sample to preserve continuity while avoiding full scans.
std::optional<double> solve_polar_angle_near_previous(
    const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
    const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
    const cpp_helper_libs::linear_algebra::UnitVector3 &axis, const double previous_polar_angle,
    const cpp_helper_libs::linear_algebra::Vector3 &direction, const double boundary_sum_radians) {
  const double delta_step = kPi / static_cast<double>(kBoundarySearchResolution);
  const double center = std::clamp(previous_polar_angle, 0.0, kPi);
  const double residual_center = boundary_residual_for_polar(focus_one, focus_two, axis, direction,
                                                             center, boundary_sum_radians);

  double best_delta = center;
  double best_abs_residual = std::fabs(residual_center);
  if (best_abs_residual <= kBoundaryApproximateResidualTolerance) {
    return center;
  }

  for (std::size_t expansion = 1U; expansion <= kBoundarySearchResolution; expansion *= 2U) {
    const double span = delta_step * static_cast<double>(expansion);
    const double lower = std::max(0.0, center - span);
    const double upper = std::min(kPi, center + span);
    const double residual_lower = boundary_residual_for_polar(focus_one, focus_two, axis, direction,
                                                              lower, boundary_sum_radians);
    const double residual_upper = boundary_residual_for_polar(focus_one, focus_two, axis, direction,
                                                              upper, boundary_sum_radians);

    const double lower_abs_residual = std::fabs(residual_lower);
    if (lower_abs_residual < best_abs_residual) {
      best_abs_residual = lower_abs_residual;
      best_delta = lower;
    }
    const double upper_abs_residual = std::fabs(residual_upper);
    if (upper_abs_residual < best_abs_residual) {
      best_abs_residual = upper_abs_residual;
      best_delta = upper;
    }

    if (lower < center && ((residual_lower == 0.0) || (residual_center == 0.0) ||
                           ((residual_lower < 0.0) != (residual_center < 0.0)))) {
      return root_from_sign_change(focus_one, focus_two, axis, direction, boundary_sum_radians,
                                   RootBracket{lower, center},
                                   BracketResiduals{residual_lower, residual_center});
    }

    if (center < upper && ((residual_center == 0.0) || (residual_upper == 0.0) ||
                           ((residual_center < 0.0) != (residual_upper < 0.0)))) {
      return root_from_sign_change(focus_one, focus_two, axis, direction, boundary_sum_radians,
                                   RootBracket{center, upper},
                                   BracketResiduals{residual_center, residual_upper});
    }

    if (lower < upper && ((residual_lower == 0.0) || (residual_upper == 0.0) ||
                          ((residual_lower < 0.0) != (residual_upper < 0.0)))) {
      return root_from_sign_change(focus_one, focus_two, axis, direction, boundary_sum_radians,
                                   RootBracket{lower, upper},
                                   BracketResiduals{residual_lower, residual_upper});
    }

    if ((lower == 0.0) && (upper == kPi)) {
      break;
    }
  }

  if (best_abs_residual <= kBoundaryApproximateResidualTolerance) {
    return best_delta;
  }

  return std::nullopt;
}

// For one azimuth, find the polar angle that satisfies the ellipse boundary equation.
// Algorithm:
// 1) Coarse scan for sign changes and near-best residual.
// 2) Refine sign-change roots using bisection.
// 3) Prefer root nearest previous angle to preserve boundary continuity.
// Assumption: neighboring azimuth samples should map to nearby boundary points.
std::optional<double>
solve_polar_angle_for_azimuth(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                              const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                              const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                              const cpp_helper_libs::linear_algebra::Vector3 &direction,
                              const double boundary_sum_radians,
                              const std::optional<double> previous_polar_angle) {
  if (previous_polar_angle.has_value()) {
    const std::optional<double> local_root = solve_polar_angle_near_previous(
        focus_one, focus_two, axis, previous_polar_angle.value(), direction, boundary_sum_radians);
    if (local_root.has_value()) {
      return local_root;
    }
  }

  const double delta_step = kPi / static_cast<double>(kBoundarySearchResolution);
  double best_delta = 0.0;
  double best_abs_residual = std::numeric_limits<double>::infinity();

  std::vector<double> roots;
  roots.reserve(4U);

  double previous_residual =
      boundary_residual_for_polar(focus_one, focus_two, axis, direction, 0.0, boundary_sum_radians);
  const double initial_abs_residual = std::fabs(previous_residual);
  if (initial_abs_residual < best_abs_residual) {
    best_abs_residual = initial_abs_residual;
    best_delta = 0.0;
  }

  for (std::size_t index = 1U; index <= kBoundarySearchResolution; ++index) {
    const double delta = delta_step * static_cast<double>(index);
    const double residual = boundary_residual_for_polar(focus_one, focus_two, axis, direction,
                                                        delta, boundary_sum_radians);
    const double abs_residual = std::fabs(residual);
    if (abs_residual < best_abs_residual) {
      best_abs_residual = abs_residual;
      best_delta = delta;
    }

    if ((previous_residual == 0.0) || (residual == 0.0) ||
        ((previous_residual < 0.0) != (residual < 0.0))) {
      const double delta_lower = delta - delta_step;
      const std::optional<double> root = root_from_sign_change(
          focus_one, focus_two, axis, direction, boundary_sum_radians,
          RootBracket{delta_lower, delta}, BracketResiduals{previous_residual, residual});
      if (root.has_value()) {
        roots.push_back(root.value());
      }
    }

    previous_residual = residual;
  }

  if (!roots.empty()) {
    if (previous_polar_angle.has_value()) {
      return *std::min_element(roots.begin(), roots.end(),
                               [&](const double left, const double right) {
                                 return std::fabs(left - previous_polar_angle.value()) <
                                        std::fabs(right - previous_polar_angle.value());
                               });
    }

    return roots.front();
  }

  if (best_abs_residual <= kBoundaryApproximateResidualTolerance) {
    return best_delta;
  }

  return std::nullopt;
}

// Build polygonal approximation of ellipse boundary as contiguous minor arcs.
// Algorithm:
// 1) Build axis-aligned local basis from the two foci.
// 2) Sample azimuths uniformly and solve boundary polar angle at each.
// 3) Connect sampled points with minor arcs.
// Assumptions:
// - Segment count is high enough to approximate boundary shape for intersection queries.
// - Ellipse parameters are valid (checked by caller).
std::optional<std::vector<MinorArc>>
build_boundary_edges(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                     const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                     const BoundaryDiscretization &discretization) {
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> axis =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(focus_one.as_vector() +
                                                                focus_two.as_vector());
  if (!axis.has_value()) {
    return std::nullopt;
  }

  cpp_helper_libs::linear_algebra::Vector3 basis_candidate =
      focus_one.as_vector() - axis->scaled_by(axis->dot(focus_one));
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> basis_from_projection =
      basis_candidate.normalized();
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> basis_first =
      basis_from_projection.has_value() ? basis_from_projection
                                        : internal::any_orthogonal_unit(axis.value());
  if (!basis_first.has_value()) {
    return std::nullopt;
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> basis_second =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(axis->cross(basis_first.value()));
  if (!basis_second.has_value()) {
    return std::nullopt;
  }

  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> boundary_points;
  boundary_points.reserve(discretization.segment_count);

  std::optional<double> previous_delta;
  for (std::size_t index = 0U; index < discretization.segment_count; ++index) {
    const double azimuth =
        kTwoPi * static_cast<double>(index) / static_cast<double>(discretization.segment_count);
    const cpp_helper_libs::linear_algebra::Vector3 direction =
        azimuth_direction(basis_first.value(), basis_second.value(), azimuth);
    const std::optional<double> solved_delta =
        solve_polar_angle_for_azimuth(focus_one, focus_two, axis.value(), direction,
                                      discretization.boundary_sum_radians, previous_delta);
    if (!solved_delta.has_value()) {
      return std::nullopt;
    }

    previous_delta = solved_delta;
    boundary_points.push_back(
        point_on_basis_with_direction(axis.value(), direction, solved_delta.value()));
  }

  std::vector<MinorArc> edges;
  edges.reserve(discretization.segment_count);
  for (std::size_t index = 0U; index < boundary_points.size(); ++index) {
    const cpp_helper_libs::linear_algebra::UnitVector3 &start = boundary_points[index];
    const cpp_helper_libs::linear_algebra::UnitVector3 &end =
        boundary_points[(index + 1U) % boundary_points.size()];

    const std::optional<MinorArc> edge = MinorArc::from_endpoints(start, end);
    if (!edge.has_value()) {
      return std::nullopt;
    }

    edges.push_back(edge.value());
  }

  return edges;
}

} // namespace

// Algorithm:
// - Validate boundary discretization count and boundary sum range.
// - Reject degenerate ellipse parameter regions using focus distance bounds.
// - Precompute a piecewise-minor-arc boundary approximation.
// - Construct the shape from validated parameters and boundary edges.
// Assumptions:
// - Focus inputs are unit radials.
// - Boundary approximation is sufficiently dense for downstream intersection queries.
std::optional<SphericalEllipse> SphericalEllipse::from_foci_and_boundary_sum(
    const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
    const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
    const cpp_helper_libs::quantities::Angle boundary_sum,
    const std::size_t boundary_segment_count) noexcept {
  if (boundary_segment_count < kMinimumBoundarySegmentCount) {
    return std::nullopt;
  }

  const double boundary_sum_radians =
      boundary_sum.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  if (boundary_sum_radians <= 0.0 || boundary_sum_radians >= kTwoPi) {
    return std::nullopt;
  }

  const double focus_distance = focus_one.central_angle_radians(focus_two);
  if (boundary_sum_radians <= (focus_distance + kContainmentTolerance)) {
    return std::nullopt;
  }

  if (boundary_sum_radians >= ((kTwoPi - focus_distance) - kContainmentTolerance)) {
    return std::nullopt;
  }

  const std::optional<std::vector<MinorArc>> boundary_edges = build_boundary_edges(
      focus_one, focus_two, BoundaryDiscretization{boundary_sum_radians, boundary_segment_count});
  if (!boundary_edges.has_value()) {
    return std::nullopt;
  }

  return SphericalEllipse(focus_one, focus_two, boundary_sum, boundary_segment_count,
                          boundary_edges.value());
}

// Algorithm:
// - Compute candidate sum of central angles to the two foci.
// - Compare candidate sum against boundary-sum threshold.
// - Use inclusive/exclusive comparator and optional tolerance slack.
// Assumptions:
// - Focus vectors are unit radials.
// - `boundary_sum_` was validated during construction.
bool SphericalEllipse::contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                       const bool inclusive, const bool exact) const noexcept {
  const double boundary_sum_radians =
      boundary_sum_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double candidate_sum =
      focus_one_.central_angle_radians(point) + focus_two_.central_angle_radians(point);

  const double tolerance = exact ? 0.0 : kContainmentTolerance;
  if (inclusive) {
    return candidate_sum <= (boundary_sum_radians + tolerance);
  }

  return candidate_sum < (boundary_sum_radians - tolerance);
}

// Algorithm:
// - Intersect the query curve with each precomputed boundary minor arc.
// - Inclusive mode returns true on any intersection record.
// - Exclusive mode requires at least one non-endpoint-touch record.
// Assumptions:
// - `boundary_edges_` forms a closed approximation of the ellipse boundary.
bool SphericalEllipse::boundary_intersects_policy(const SphericalCurve &curve, const bool inclusive,
                                                  const bool exact) const noexcept {
  return std::any_of(boundary_edges_.begin(), boundary_edges_.end(), [&](const MinorArc &edge) {
    return internal::curves_intersect(edge, curve, exact, inclusive);
  });
}

} // namespace cpp_helper_libs::spherical_geometry
