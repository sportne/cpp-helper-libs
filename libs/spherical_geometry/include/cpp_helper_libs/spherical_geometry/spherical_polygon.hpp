// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP

#include <optional>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_shape.hpp"

namespace cpp_helper_libs::spherical_geometry {

class SphericalCurve;

/**
 * @brief Non-self-intersecting spherical polygon made of minor-arc edges.
 */
class SphericalPolygon final : public SphericalShape {
public:
  static std::optional<SphericalPolygon>
  from_vertices(const std::vector<cpp_helper_libs::linear_algebra::UnitVector3> &vertices) noexcept;

  const std::vector<cpp_helper_libs::linear_algebra::UnitVector3> &vertices() const noexcept {
    return vertices_;
  }

  bool contains_inclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  bool contains_exclusive(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  bool contains_inclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;
  bool contains_exclusive_exact(
      const cpp_helper_libs::linear_algebra::UnitVector3 &point) const noexcept override;

  bool boundary_intersects_inclusive(const SphericalCurve &curve) const noexcept override;
  bool boundary_intersects_exclusive(const SphericalCurve &curve) const noexcept override;
  bool boundary_intersects_inclusive_exact(const SphericalCurve &curve) const noexcept override;
  bool boundary_intersects_exclusive_exact(const SphericalCurve &curve) const noexcept override;

private:
  SphericalPolygon(std::vector<cpp_helper_libs::linear_algebra::UnitVector3> vertices,
                   std::vector<MinorArc> edges) noexcept
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  bool contains_policy(const cpp_helper_libs::linear_algebra::UnitVector3 &point, bool inclusive,
                       bool exact) const noexcept;

  bool boundary_intersects_policy(const SphericalCurve &curve, bool inclusive,
                                  bool exact) const noexcept;

  /// Polygon vertices in traversal order around the boundary.
  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> vertices_;
  /// Boundary edges as minor-arc segments between adjacent vertices.
  std::vector<MinorArc> edges_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_POLYGON_HPP
