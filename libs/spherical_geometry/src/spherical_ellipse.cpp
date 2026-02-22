// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_ellipse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
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

struct BoundaryDiscretization final {
  // Target sum of distances to the two foci for boundary points.
  double boundary_sum_radians;
  // Number of boundary samples/segments used in the polygonal approximation.
  std::size_t segment_count;
};

// Convert local spherical coordinates (polar angle + azimuth around an axis basis)
// into a unit radial on the sphere.
// Assumption: axis/basis_first/basis_second form an orthonormal frame.
cpp_helper_libs::linear_algebra::UnitVector3
point_on_basis(const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
               const cpp_helper_libs::linear_algebra::UnitVector3 &basis_first,
               const cpp_helper_libs::linear_algebra::UnitVector3 &basis_second,
               const double polar_angle, const double azimuth) {
  const cpp_helper_libs::linear_algebra::Vector3 direction =
      (basis_first.scaled_by(std::cos(azimuth)) + basis_second.scaled_by(std::sin(azimuth)));
  const cpp_helper_libs::linear_algebra::Vector3 point =
      axis.scaled_by(std::cos(polar_angle)) + direction * std::sin(polar_angle);

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

// Residual function for the ellipse boundary equation:
// angle(focus_one, point) + angle(focus_two, point) - boundary_sum = 0.
double boundary_residual(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                         const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                         const double boundary_sum_radians) {
  return focus_one.central_angle_radians(point) + focus_two.central_angle_radians(point) -
         boundary_sum_radians;
}

// Solve one boundary root inside a bracket where the residual changes sign.
// Algorithm: bisection with fixed iteration cap and residual tolerance.
std::optional<double>
root_from_sign_change(const cpp_helper_libs::linear_algebra::UnitVector3 &focus_one,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &focus_two,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &basis_first,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &basis_second,
                      const double azimuth, const double boundary_sum_radians,
                      const RootBracket bracket) {
  double lower = bracket.lower;
  double upper = bracket.upper;

  double residual_lower = boundary_residual(
      focus_one, focus_two, point_on_basis(axis, basis_first, basis_second, lower, azimuth),
      boundary_sum_radians);
  const double residual_upper = boundary_residual(
      focus_one, focus_two, point_on_basis(axis, basis_first, basis_second, upper, azimuth),
      boundary_sum_radians);

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
    const double residual_midpoint = boundary_residual(
        focus_one, focus_two, point_on_basis(axis, basis_first, basis_second, midpoint, azimuth),
        boundary_sum_radians);

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
                              const cpp_helper_libs::linear_algebra::UnitVector3 &basis_first,
                              const cpp_helper_libs::linear_algebra::UnitVector3 &basis_second,
                              const double azimuth, const double boundary_sum_radians,
                              const std::optional<double> previous_polar_angle) {
  double best_delta = 0.0;
  double best_abs_residual = std::numeric_limits<double>::infinity();

  std::vector<double> roots;
  roots.reserve(4U);

  double previous_residual = 0.0;
  bool have_previous_residual = false;
  for (std::size_t index = 0U; index <= kBoundarySearchResolution; ++index) {
    const double delta =
        kPi * static_cast<double>(index) / static_cast<double>(kBoundarySearchResolution);
    const double residual = boundary_residual(
        focus_one, focus_two, point_on_basis(axis, basis_first, basis_second, delta, azimuth),
        boundary_sum_radians);
    const double abs_residual = std::fabs(residual);
    if (abs_residual < best_abs_residual) {
      best_abs_residual = abs_residual;
      best_delta = delta;
    }

    if (have_previous_residual && ((previous_residual == 0.0) || (residual == 0.0) ||
                                   ((previous_residual < 0.0) != (residual < 0.0)))) {
      const double delta_lower =
          kPi * static_cast<double>(index - 1U) / static_cast<double>(kBoundarySearchResolution);
      const double delta_upper = delta;
      const std::optional<double> root =
          root_from_sign_change(focus_one, focus_two, axis, basis_first, basis_second, azimuth,
                                boundary_sum_radians, RootBracket{delta_lower, delta_upper});
      if (root.has_value()) {
        roots.push_back(root.value());
      }
    }

    previous_residual = residual;
    have_previous_residual = true;
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
    const std::optional<double> solved_delta = solve_polar_angle_for_azimuth(
        focus_one, focus_two, axis.value(), basis_first.value(), basis_second.value(), azimuth,
        discretization.boundary_sum_radians, previous_delta);
    if (!solved_delta.has_value()) {
      return std::nullopt;
    }

    previous_delta = solved_delta;
    boundary_points.push_back(point_on_basis(axis.value(), basis_first.value(),
                                             basis_second.value(), solved_delta.value(), azimuth));
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
// - Delegate to shared containment policy helper with inclusive+tolerant settings.
// Assumptions:
// - Tolerant mode is the default public behavior for containment.
bool SphericalEllipse::contains_inclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, true, false);
}

// Algorithm:
// - Delegate to shared containment policy helper with exclusive+tolerant settings.
// Assumptions:
// - Boundary points are excluded in exclusive mode.
bool SphericalEllipse::contains_exclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, false, false);
}

// Algorithm:
// - Delegate to shared containment policy helper with inclusive+exact settings.
// Assumptions:
// - Exact mode performs strict comparisons without tolerance slack.
bool SphericalEllipse::contains_inclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, true, true);
}

// Algorithm:
// - Delegate to shared containment policy helper with exclusive+exact settings.
// Assumptions:
// - Exact exclusive mode excludes points exactly on the boundary.
bool SphericalEllipse::contains_exclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  return contains_policy(point, false, true);
}

// Algorithm:
// - Delegate to shared boundary-intersection helper with inclusive+tolerant settings.
// Assumptions:
// - Endpoint touches count as intersections in inclusive mode.
bool SphericalEllipse::boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, true, false);
}

// Algorithm:
// - Delegate to shared boundary-intersection helper with exclusive+tolerant settings.
// Assumptions:
// - Endpoint-only contacts do not count in exclusive mode.
bool SphericalEllipse::boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, false, false);
}

// Algorithm:
// - Delegate to shared boundary-intersection helper with inclusive+exact settings.
// Assumptions:
// - Exact mode uses strict comparisons in underlying curve intersections.
bool SphericalEllipse::boundary_intersects_inclusive_exact(
    const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, true, true);
}

// Algorithm:
// - Delegate to shared boundary-intersection helper with exclusive+exact settings.
// Assumptions:
// - Endpoint-only contacts do not count in exclusive mode.
bool SphericalEllipse::boundary_intersects_exclusive_exact(
    const SphericalCurve &curve) const noexcept {
  return boundary_intersects_policy(curve, false, true);
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
  const auto intersections_for_edge = [&](const MinorArc &edge) {
    const std::vector<CurveIntersection> intersections =
        exact ? edge.intersections_with_exact(curve) : edge.intersections_with(curve);
    return intersections;
  };

  if (inclusive) {
    return std::any_of(boundary_edges_.begin(), boundary_edges_.end(),
                       [&](const MinorArc &edge) { return !intersections_for_edge(edge).empty(); });
  }

  return std::any_of(boundary_edges_.begin(), boundary_edges_.end(), [&](const MinorArc &edge) {
    const std::vector<CurveIntersection> intersections = intersections_for_edge(edge);
    return std::any_of(intersections.begin(), intersections.end(),
                       [](const CurveIntersection &intersection) {
                         return intersection.kind() != CurveIntersectionKind::EndpointTouch;
                       });
  });
}

} // namespace cpp_helper_libs::spherical_geometry
