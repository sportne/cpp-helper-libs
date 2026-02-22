// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CURVE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CURVE_HPP

#include <optional>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Abstract curve on the unit sphere.
 */
class SphericalCurve {
public:
  virtual ~SphericalCurve() = default;

  /**
   * @brief Radial direction of the curve's first point.
   *
   * @return Unit radial on the sphere surface.
   */
  virtual cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept = 0;
  /**
   * @brief Radial direction of the curve's last point.
   *
   * @return Unit radial on the sphere surface.
   */
  virtual cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept = 0;

  /**
   * @brief Oriented frame at the start of the curve.
   *
   * The returned frame radial must equal @ref start_radial.
   */
  virtual SphericalRay start_ray() const noexcept = 0;
  /**
   * @brief Oriented frame at the end of the curve.
   *
   * The returned frame radial must equal @ref end_radial.
   */
  virtual SphericalRay end_ray() const noexcept = 0;

  /**
   * @brief Arc length measured along the sphere surface.
   *
   * Returned value is an angle on the unit sphere.
   */
  virtual cpp_helper_libs::quantities::Angle length() const noexcept = 0;

  /**
   * @brief Compute curve-curve intersections using tolerant floating-point policy.
   *
   * @param other Other curve to intersect with.
   * @return Zero or more intersection records, each carrying geometry and per-curve location
   * metadata.
   */
  std::vector<CurveIntersection> intersections_with(const SphericalCurve &other) const noexcept;

  /**
   * @brief Compute curve-curve intersections using exact comparison policy.
   *
   * @param other Other curve to intersect with.
   * @return Zero or more intersection records, each carrying geometry and per-curve location
   * metadata.
   */
  std::vector<CurveIntersection>
  intersections_with_exact(const SphericalCurve &other) const noexcept;

public:
  /**
   * @brief Support-axis direction of the curve's underlying circle.
   *
   * Internal contract used by intersection algorithms.
   * For great circles this is the plane normal.
   */
  virtual cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept = 0;
  /**
   * @brief Support-circle constant `axis.dot(point) = constant`.
   *
   * For great circles this is `0`; for small circles it is `cos(radius)`.
   */
  virtual double support_constant() const noexcept = 0;
  /**
   * @brief Signed sweep in radians used to parameterize curve traversal.
   *
   * Positive/negative sign encodes traversal direction around support axis.
   * Zero indicates a degenerate point-like curve.
   */
  virtual double signed_sweep_radians() const noexcept = 0;

  /**
   * @brief Locate a candidate point on this curve.
   *
   * @param point Candidate unit radial on the sphere.
   * @param exact Comparison policy selector.
   * @return Curve location metadata when point lies on the curve, else `std::nullopt`.
   */
  virtual std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept = 0;

  /**
   * @brief Evaluate the curve at normalized parameter `t` in [0, 1].
   *
   * Implementations must clamp or otherwise safely handle out-of-range values.
   */
  virtual cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept = 0;

  /**
   * @brief Identify whether this curve is a degenerate single-point curve.
   */
  virtual bool is_zero_length_curve() const noexcept = 0;

private:
  /**
   * @brief Shared implementation for tolerant/exact intersection queries.
   *
   * @param exact `true` for strict floating-point comparisons, `false` for tolerant behavior.
   */
  std::vector<CurveIntersection> intersections_with_policy(const SphericalCurve &other,
                                                           bool exact) const noexcept;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CURVE_HPP
