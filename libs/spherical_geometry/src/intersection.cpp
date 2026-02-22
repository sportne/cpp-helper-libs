// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/intersection.hpp"

namespace cpp_helper_libs::spherical_geometry {

CurveIntersection
CurveIntersection::point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                         const CurveLocation &first_curve_location,
                         const CurveLocation &second_curve_location) noexcept {
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
  return {CurveIntersectionKind::OverlapSegment, first_point,  first_curve_first_location,
          second_curve_first_location,           second_point, first_curve_second_location,
          second_curve_second_location};
}

} // namespace cpp_helper_libs::spherical_geometry
