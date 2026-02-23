// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CIRCLE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CIRCLE_HPP

#include <optional>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/detail/policy_shape_facade.hpp"
#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

/**
 * @brief Small circle on the unit sphere.
 */
class SphericalCircle final : public detail::PolicyShapeFacade<SphericalCircle> {
public:
  /**
   * @brief Create a spherical circle from center direction and radius.
   *
   * @param center Unit radial pointing to circle center direction.
   * @param radius Central-angle radius from center to boundary.
   * @return Circle on valid radius `(0, pi)`, else `std::nullopt`.
   */
  static std::optional<SphericalCircle>
  from_center_and_radius(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                         cpp_helper_libs::quantities::Angle radius) noexcept;

  /**
   * @brief Center direction of the circle on the unit sphere.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 center() const noexcept { return center_; }
  /**
   * @brief Circle radius represented as central angle.
   */
  cpp_helper_libs::quantities::Angle radius() const noexcept { return radius_; }

private:
  template <typename> friend class detail::PolicyShapeFacade;

  SphericalCircle(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                  cpp_helper_libs::quantities::Angle radius,
                  std::vector<SmallArc> boundary_arcs) noexcept
      : center_(center), radius_(radius), boundary_arcs_(std::move(boundary_arcs)) {}

  /**
   * @brief Shared containment evaluator for inclusive/exclusive and tolerant/exact variants.
   */
  bool contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point, bool inclusive,
                       bool exact) const noexcept;

  /**
   * @brief Shared boundary-intersection evaluator for inclusive/exclusive and tolerant/exact
   * variants.
   */
  bool boundary_intersects_policy(const SphericalCurve &curve, bool inclusive,
                                  bool exact) const noexcept;

  /// Unit radial pointing from the sphere origin to the circle center.
  cpp_helper_libs::linear_algebra::UnitVector3 center_;
  /// Circle radius as a central angle from @ref center_ to the boundary.
  cpp_helper_libs::quantities::Angle radius_;
  /// Two half-circle arcs used to represent the full boundary for intersection checks.
  std::vector<SmallArc> boundary_arcs_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_CIRCLE_HPP
