// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_polygon.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/zero_length_curve.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
class SphericalCurve;
} // namespace cpp_helper_libs::spherical_geometry

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi in radians; winding angle above this implies interior for convex orientation test.
constexpr double kPi = std::numbers::pi_v<double>;
// Tolerance for robust boundary/inside checks under floating-point arithmetic.
constexpr double kBoundaryTolerance = 1e-10;

bool edges_adjacent(const std::size_t first_index, const std::size_t second_index,
                    const std::size_t edge_count) {
  // Algorithm:
  // - Treat identical indices as adjacent (same edge).
  // - Treat cyclic predecessor/successor indices as adjacent.
  // Assumptions:
  // - Edges are ordered around a closed polygon ring.
  if (first_index == second_index) {
    return true;
  }

  if ((first_index + 1U) % edge_count == second_index) {
    return true;
  }

  if ((second_index + 1U) % edge_count == first_index) {
    return true;
  }

  return false;
}

bool point_on_boundary(const std::vector<MinorArc> &edges,
                       const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                       const bool exact) {
  // Algorithm:
  // - Wrap the query point as a zero-length curve.
  // - Intersect each boundary edge with that point-curve.
  // - Report true when any edge contains the point.
  // Assumptions:
  // - Curve intersection engine provides robust endpoint handling.
  const std::optional<ZeroLengthCurve> point_curve = ZeroLengthCurve::at_radial(point);
  if (!point_curve.has_value()) {
    return false;
  }

  return std::any_of(edges.begin(), edges.end(), [&](const MinorArc &edge) {
    const std::vector<CurveIntersection> intersections =
        exact ? edge.intersections_with_exact(point_curve.value())
              : edge.intersections_with(point_curve.value());
    return !intersections.empty();
  });
}

} // namespace

// Algorithm:
// - Require at least three vertices (after optional duplicate closing vertex removal).
// - Build each side as a MinorArc between consecutive vertices.
// - Check every non-adjacent edge pair for self-intersections:
//   reject overlap segments or interior crossings.
// - Accept endpoint-only contacts for adjacent connectivity.
// Assumptions:
// - Input order defines polygon boundary traversal direction.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<SphericalPolygon> SphericalPolygon::from_vertices(
    const std::vector<cpp_helper_libs::linear_algebra::UnitVector3> &vertices) noexcept {
  if (vertices.size() < 3U) {
    return std::nullopt;
  }

  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> normalized_vertices = vertices;
  if (internal::same_radial(normalized_vertices.front(), normalized_vertices.back(),
                            internal::tolerant_policy())) {
    normalized_vertices.pop_back();
  }

  if (normalized_vertices.size() < 3U) {
    return std::nullopt;
  }

  std::vector<MinorArc> edges;
  edges.reserve(normalized_vertices.size());

  for (std::size_t index = 0U; index < normalized_vertices.size(); ++index) {
    const cpp_helper_libs::linear_algebra::UnitVector3 &start = normalized_vertices[index];
    const cpp_helper_libs::linear_algebra::UnitVector3 &end =
        normalized_vertices[(index + 1U) % normalized_vertices.size()];

    const std::optional<MinorArc> edge = MinorArc::from_endpoints(start, end);
    if (!edge.has_value()) {
      return std::nullopt;
    }

    edges.push_back(edge.value());
  }

  for (std::size_t first_index = 0U; first_index < edges.size(); ++first_index) {
    for (std::size_t second_index = first_index + 1U; second_index < edges.size(); ++second_index) {
      if (edges_adjacent(first_index, second_index, edges.size())) {
        continue;
      }

      const std::vector<CurveIntersection> intersections =
          edges[first_index].intersections_with(edges[second_index]);
      for (const CurveIntersection &intersection : intersections) {
        if (intersection.kind() == CurveIntersectionKind::OverlapSegment) {
          return std::nullopt;
        }

        const bool at_first_endpoint = intersection.first_curve_first_location().at_start ||
                                       intersection.first_curve_first_location().at_end;
        const bool at_second_endpoint = intersection.second_curve_first_location().at_start ||
                                        intersection.second_curve_first_location().at_end;
        if (!(at_first_endpoint && at_second_endpoint)) {
          return std::nullopt;
        }
      }
    }
  }

  return SphericalPolygon(std::move(normalized_vertices), std::move(edges));
}

bool SphericalPolygon::contains_inclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to winding-based containment with inclusive boundary mode.
  // Assumptions:
  // - Default mode uses tolerant numerics.
  return contains_policy(point, true, false);
}

bool SphericalPolygon::contains_exclusive(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to winding-based containment with exclusive boundary mode.
  // Assumptions:
  // - Boundary points are excluded.
  return contains_policy(point, false, false);
}

bool SphericalPolygon::contains_inclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to winding-based containment with exact + inclusive semantics.
  // Assumptions:
  // - Exact mode uses strict thresholding on winding sum.
  return contains_policy(point, true, true);
}

bool SphericalPolygon::contains_exclusive_exact(
    const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept {
  // Algorithm:
  // - Delegate to winding-based containment with exact + exclusive semantics.
  // Assumptions:
  // - Exact mode has no tolerance buffer near the boundary.
  return contains_policy(point, false, true);
}

bool SphericalPolygon::boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to edge-intersection policy with inclusive semantics.
  // Assumptions:
  // - Endpoint touches count in inclusive mode.
  return boundary_intersects_policy(curve, true, false);
}

bool SphericalPolygon::boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to edge-intersection policy with exclusive semantics.
  // Assumptions:
  // - Endpoint-only touches are excluded.
  return boundary_intersects_policy(curve, false, false);
}

bool SphericalPolygon::boundary_intersects_inclusive_exact(
    const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to exact edge-intersection policy with inclusive semantics.
  // Assumptions:
  // - Exact mode disables tolerance in curve intersection calculations.
  return boundary_intersects_policy(curve, true, true);
}

bool SphericalPolygon::boundary_intersects_exclusive_exact(
    const SphericalCurve &curve) const noexcept {
  // Algorithm:
  // - Delegate to exact edge-intersection policy with exclusive semantics.
  // Assumptions:
  // - Endpoint-only events are filtered out.
  return boundary_intersects_policy(curve, false, true);
}

bool SphericalPolygon::contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                       const bool inclusive, const bool exact) const noexcept {
  // Algorithm:
  // - First test whether point lies on boundary; return based on inclusivity.
  // - Otherwise compute spherical winding around the query point:
  //   project consecutive vertices onto tangent plane at point, normalize, and accumulate signed
  //   turning angles.
  // - Compare accumulated winding against pi threshold to classify interior.
  // Assumptions:
  // - Polygon is simple and represented by MinorArc edges created at construction.
  if (point_on_boundary(edges_, point, exact)) {
    return inclusive;
  }

  double signed_winding_sum = 0.0;
  for (std::size_t index = 0U; index < vertices_.size(); ++index) {
    const cpp_helper_libs::linear_algebra::UnitVector3 &vertex_first = vertices_[index];
    const cpp_helper_libs::linear_algebra::UnitVector3 &vertex_second =
        vertices_[(index + 1U) % vertices_.size()];

    const cpp_helper_libs::linear_algebra::Vector3 projected_first =
        vertex_first.as_vector() - point.scaled_by(point.dot(vertex_first));
    const cpp_helper_libs::linear_algebra::Vector3 projected_second =
        vertex_second.as_vector() - point.scaled_by(point.dot(vertex_second));

    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> unit_first =
        projected_first.normalized();
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> unit_second =
        projected_second.normalized();

    if (!unit_first.has_value() || !unit_second.has_value()) {
      return inclusive;
    }

    const double signed_angle =
        std::atan2(point.dot(unit_first->cross(unit_second.value())),
                   internal::clamp_cosine(unit_first->dot(unit_second.value())));
    signed_winding_sum += signed_angle;
  }

  if (exact) {
    return signed_winding_sum > kPi;
  }

  return signed_winding_sum > (kPi - kBoundaryTolerance);
}

bool SphericalPolygon::boundary_intersects_policy(const SphericalCurve &curve, const bool inclusive,
                                                  const bool exact) const noexcept {
  // Algorithm:
  // - Intersect query curve with each polygon edge.
  // - Inclusive mode accepts any intersection.
  // - Exclusive mode requires at least one intersection that is not an endpoint touch.
  // Assumptions:
  // - Boundary is fully represented by `edges_` in order.
  return std::any_of(edges_.begin(), edges_.end(), [&](const MinorArc &edge) {
    const std::vector<CurveIntersection> intersections =
        exact ? edge.intersections_with_exact(curve) : edge.intersections_with(curve);

    if (intersections.empty()) {
      return false;
    }

    if (inclusive) {
      return true;
    }

    return std::any_of(intersections.begin(), intersections.end(),
                       [](const CurveIntersection &intersection) {
                         return intersection.kind() != CurveIntersectionKind::EndpointTouch;
                       });
  });
}

} // namespace cpp_helper_libs::spherical_geometry
