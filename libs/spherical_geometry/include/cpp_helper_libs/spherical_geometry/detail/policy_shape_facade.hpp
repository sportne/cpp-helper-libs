// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_DETAIL_POLICY_SHAPE_FACADE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_DETAIL_POLICY_SHAPE_FACADE_HPP

#include "cpp_helper_libs/spherical_geometry/spherical_shape.hpp"

namespace cpp_helper_libs::spherical_geometry::detail {

/**
 * @brief CRTP facade that provides all `SphericalShape` wrapper policy methods.
 *
 * Derived types are expected to implement:
 * - `bool contains_policy(const UnitVector3&, bool inclusive, bool exact) const noexcept`
 * - `bool boundary_intersects_policy(const SphericalCurve&, bool inclusive, bool exact) const
 * noexcept`
 */
template <typename Derived> class PolicyShapeFacade : public SphericalShape {
public:
  bool contains_inclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept final {
    return derived().contains_policy(point, true, false);
  }

  bool contains_exclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept final {
    return derived().contains_policy(point, false, false);
  }

  bool contains_inclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept final {
    return derived().contains_policy(point, true, true);
  }

  bool contains_exclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept final {
    return derived().contains_policy(point, false, true);
  }

  bool boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept final {
    return derived().boundary_intersects_policy(curve, true, false);
  }

  bool boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept final {
    return derived().boundary_intersects_policy(curve, false, false);
  }

  bool boundary_intersects_inclusive_exact(const SphericalCurve &curve) const noexcept final {
    return derived().boundary_intersects_policy(curve, true, true);
  }

  bool boundary_intersects_exclusive_exact(const SphericalCurve &curve) const noexcept final {
    return derived().boundary_intersects_policy(curve, false, true);
  }

private:
  const Derived &derived() const noexcept { return static_cast<const Derived &>(*this); }
};

} // namespace cpp_helper_libs::spherical_geometry::detail

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_DETAIL_POLICY_SHAPE_FACADE_HPP
