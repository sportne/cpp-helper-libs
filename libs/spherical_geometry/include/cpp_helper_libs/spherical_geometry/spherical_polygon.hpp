// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP

#include <optional>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/detail/policy_shape_facade.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

/**
 * @brief Non-self-intersecting spherical polygon made of minor-arc edges.
 */
class SphericalPolygon final : public detail::PolicyShapeFacade<SphericalPolygon> {
public:
  /**
   * @brief Build a simple spherical polygon from ordered boundary vertices.
   *
   * @param vertices Boundary vertices in traversal order. A repeated final vertex equal to the
   * first is accepted and removed.
   * @return Polygon when edges are valid minor arcs and the boundary does not self-intersect;
   * otherwise `std::nullopt`.
   */
  static std::optional<SphericalPolygon>
  from_vertices(const std::vector<cpp_helper_libs::linear_algebra::UnitVector3> &vertices) noexcept;

  /**
   * @brief Read-only access to normalized polygon vertices in traversal order.
   */
  const std::vector<cpp_helper_libs::linear_algebra::UnitVector3> &vertices() const noexcept {
    return vertices_;
  }

private:
  template <typename> friend class detail::PolicyShapeFacade;

  SphericalPolygon(std::vector<cpp_helper_libs::linear_algebra::UnitVector3> vertices,
                   std::vector<MinorArc> edges) noexcept
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

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

  /// Polygon vertices in traversal order around the boundary.
  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> vertices_;
  /// Boundary edges as minor-arc segments between adjacent vertices.
  std::vector<MinorArc> edges_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP
