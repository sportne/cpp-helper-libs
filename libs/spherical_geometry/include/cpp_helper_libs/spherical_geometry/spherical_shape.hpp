// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_SHAPE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_SHAPE_HPP

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

/**
 * @brief Abstract spherical shape with containment and boundary intersection queries.
 */
class SphericalShape {
public:
  virtual ~SphericalShape() = default;

  /**
   * @brief Test whether @p point is inside the shape or on its boundary.
   *
   * Tolerant floating-point policy.
   * @return `true` when point is in interior or on boundary.
   */
  virtual bool
  contains_inclusive(const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  /**
   * @brief Test whether @p point is strictly inside the shape.
   *
   * Tolerant floating-point policy.
   * @return `true` only for interior points.
   */
  virtual bool
  contains_exclusive(const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  /**
   * @brief Exact variant of @ref contains_inclusive.
   *
   * @return `true` when point is in interior or on boundary under exact comparisons.
   */
  virtual bool contains_inclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  /**
   * @brief Exact variant of @ref contains_exclusive.
   *
   * @return `true` only for strict interior under exact comparisons.
   */
  virtual bool contains_exclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  /**
   * @brief Test whether @p curve intersects the boundary, including endpoint touches.
   *
   * Tolerant floating-point policy.
   * @return `true` when any boundary contact exists.
   */
  virtual bool boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept = 0;

  /**
   * @brief Test whether @p curve intersects the boundary with non-endpoint overlap/crossing.
   *
   * Tolerant floating-point policy.
   * @return `true` when intersection is stronger than endpoint-only contact.
   */
  virtual bool boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept = 0;

  /**
   * @brief Exact variant of @ref boundary_intersects_inclusive.
   *
   * @return `true` when any boundary contact exists under exact comparisons.
   */
  virtual bool boundary_intersects_inclusive_exact(const SphericalCurve &curve) const noexcept = 0;

  /**
   * @brief Exact variant of @ref boundary_intersects_exclusive.
   *
   * @return `true` when non-endpoint intersection exists under exact comparisons.
   */
  virtual bool boundary_intersects_exclusive_exact(const SphericalCurve &curve) const noexcept = 0;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_SHAPE_HPP
