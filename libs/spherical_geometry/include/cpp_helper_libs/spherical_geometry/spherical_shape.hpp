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

  virtual bool
  contains_inclusive(const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  virtual bool
  contains_exclusive(const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  virtual bool contains_inclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  virtual bool contains_exclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept = 0;

  virtual bool boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept = 0;

  virtual bool boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept = 0;

  virtual bool boundary_intersects_inclusive_exact(const SphericalCurve &curve) const noexcept = 0;

  virtual bool boundary_intersects_exclusive_exact(const SphericalCurve &curve) const noexcept = 0;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_SHAPE_HPP
