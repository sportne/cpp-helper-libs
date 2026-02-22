// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/intersection.hpp"

namespace cpp_helper_libs::spherical_geometry {

CurveIntersection
CurveIntersection::point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                         const CurveLocation &first_curve_location,
                         const CurveLocation &second_curve_location) noexcept {
  // Algorithm:
  // - Build a point-type intersection record.
  // - Store only one geometric point and the per-curve locations at that point.
  // Assumptions:
  // - Callers provide consistent location metadata for both curves.
  return {CurveIntersectionKind::Point,
          point,
          first_curve_location,
          second_curve_location,
          std::nullopt,
          std::nullopt,
          std::nullopt};
}

CurveIntersection
CurveIntersection::endpoint_touch(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                  const CurveLocation &first_curve_location,
                                  const CurveLocation &second_curve_location) noexcept {
  // Algorithm:
  // - Build an endpoint-touch record using the same payload shape as point intersections.
  // - Differentiate semantics only through the `EndpointTouch` enum kind.
  // Assumptions:
  // - At least one location corresponds to a curve endpoint.
  return {CurveIntersectionKind::EndpointTouch,
          point,
          first_curve_location,
          second_curve_location,
          std::nullopt,
          std::nullopt,
          std::nullopt};
}

CurveIntersection
CurveIntersection::overlap_segment(const cpp_helper_libs::linear_algebra::UnitVector3 &first_point,
                                   const cpp_helper_libs::linear_algebra::UnitVector3 &second_point,
                                   const CurveLocation &first_curve_first_location,
                                   const CurveLocation &first_curve_second_location,
                                   const CurveLocation &second_curve_first_location,
                                   const CurveLocation &second_curve_second_location) noexcept {
  // Algorithm:
  // - Build an overlap-segment record with two boundary points.
  // - Store location metadata for both curves at each boundary point.
  // Assumptions:
  // - `first_point` and `second_point` delimit the same geometric overlap on both curves.
  return {CurveIntersectionKind::OverlapSegment, first_point,  first_curve_first_location,
          second_curve_first_location,           second_point, first_curve_second_location,
          second_curve_second_location};
}

} // namespace cpp_helper_libs::spherical_geometry
