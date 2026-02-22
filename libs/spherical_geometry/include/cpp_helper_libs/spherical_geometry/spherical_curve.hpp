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

  virtual cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept = 0;
  virtual cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept = 0;

  virtual SphericalRay start_ray() const noexcept = 0;
  virtual SphericalRay end_ray() const noexcept = 0;

  virtual cpp_helper_libs::quantities::Angle length() const noexcept = 0;

  /**
   * @brief Compute curve-curve intersections using tolerant floating-point policy.
   */
  std::vector<CurveIntersection> intersections_with(const SphericalCurve &other) const noexcept;

  /**
   * @brief Compute curve-curve intersections using exact comparison policy.
   */
  std::vector<CurveIntersection>
  intersections_with_exact(const SphericalCurve &other) const noexcept;

public:
  // Internal geometric hooks used for cross-curve intersection algorithms.
  virtual cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept = 0;
  virtual double support_constant() const noexcept = 0;
  virtual double signed_sweep_radians() const noexcept = 0;

  virtual std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept = 0;

  virtual cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept = 0;

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
