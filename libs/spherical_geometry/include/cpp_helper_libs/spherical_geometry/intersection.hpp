// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_INTERSECTION_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_INTERSECTION_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Location metadata for an intersection point on a curve.
 */
struct CurveLocation final {
  /// Normalized parameter on [0, 1] where the intersection lies on the curve.
  double parameter;
  /// True when the intersection is at the curve's start endpoint.
  bool at_start;
  /// True when the intersection is at the curve's end endpoint.
  bool at_end;
};

/**
 * @brief Classification for a curve-curve intersection record.
 */
enum class CurveIntersectionKind {
  Point,
  EndpointTouch,
  OverlapSegment,
};

/**
 * @brief Intersection record for spherical curves.
 *
 * For point-like intersections, only `first_point` and first locations are used.
 * For overlap intersections, `second_point` and second locations are populated.
 */
class CurveIntersection final {
public:
  static CurveIntersection point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                 const CurveLocation &first_curve_location,
                                 const CurveLocation &second_curve_location) noexcept;

  static CurveIntersection endpoint_touch(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                          const CurveLocation &first_curve_location,
                                          const CurveLocation &second_curve_location) noexcept;

  static CurveIntersection
  overlap_segment(const cpp_helper_libs::linear_algebra::UnitVector3 &first_point,
                  const cpp_helper_libs::linear_algebra::UnitVector3 &second_point,
                  const CurveLocation &first_curve_first_location,
                  const CurveLocation &first_curve_second_location,
                  const CurveLocation &second_curve_first_location,
                  const CurveLocation &second_curve_second_location) noexcept;

  CurveIntersectionKind kind() const noexcept { return kind_; }

  cpp_helper_libs::linear_algebra::UnitVector3 first_point() const noexcept { return first_point_; }

  std::optional<cpp_helper_libs::linear_algebra::UnitVector3> second_point() const noexcept {
    return second_point_;
  }

  CurveLocation first_curve_first_location() const noexcept { return first_curve_first_location_; }

  std::optional<CurveLocation> first_curve_second_location() const noexcept {
    return first_curve_second_location_;
  }

  CurveLocation second_curve_first_location() const noexcept {
    return second_curve_first_location_;
  }

  std::optional<CurveLocation> second_curve_second_location() const noexcept {
    return second_curve_second_location_;
  }

private:
  CurveIntersection(CurveIntersectionKind kind,
                    const cpp_helper_libs::linear_algebra::UnitVector3 &first_point,
                    const CurveLocation &first_curve_first_location,
                    const CurveLocation &second_curve_first_location,
                    std::optional<cpp_helper_libs::linear_algebra::UnitVector3> second_point,
                    std::optional<CurveLocation> first_curve_second_location,
                    std::optional<CurveLocation> second_curve_second_location) noexcept
      : kind_(kind), first_point_(first_point),
        first_curve_first_location_(first_curve_first_location),
        second_curve_first_location_(second_curve_first_location), second_point_(second_point),
        first_curve_second_location_(first_curve_second_location),
        second_curve_second_location_(second_curve_second_location) {}

  /// Category of this intersection record (point, endpoint touch, or overlap).
  CurveIntersectionKind kind_;
  /// First point of the intersection geometry.
  cpp_helper_libs::linear_algebra::UnitVector3 first_point_;
  /// First curve location corresponding to @ref first_point_.
  CurveLocation first_curve_first_location_;
  /// Second curve location corresponding to @ref first_point_.
  CurveLocation second_curve_first_location_;
  /// Optional second point (present for overlap segments).
  std::optional<cpp_helper_libs::linear_algebra::UnitVector3> second_point_;
  /// Optional first-curve location for @ref second_point_.
  std::optional<CurveLocation> first_curve_second_location_;
  /// Optional second-curve location for @ref second_point_.
  std::optional<CurveLocation> second_curve_second_location_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_INTERSECTION_HPP
