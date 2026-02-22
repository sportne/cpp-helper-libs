// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_RAY_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_RAY_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"

namespace cpp_helper_libs::quantities {
class Angle;
} // namespace cpp_helper_libs::quantities

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Orthonormal frame attached to a point on the unit sphere.
 *
 * Invariant: `normal == radial x tangent`.
 */
class SphericalRay final {
public:
  /**
   * @brief Construct from explicit orthonormal frame vectors.
   *
   * @return Valid ray if frame satisfies orthonormal and handedness constraints.
   */
  static std::optional<SphericalRay>
  from_frame(const cpp_helper_libs::linear_algebra::UnitVector3 &radial,
             const cpp_helper_libs::linear_algebra::UnitVector3 &normal,
             const cpp_helper_libs::linear_algebra::UnitVector3 &tangent) noexcept;

  /**
   * @brief Construct from radial and tangent vectors.
   *
   * Normal is inferred as `radial x tangent`.
   */
  static std::optional<SphericalRay>
  from_radial_and_tangent(const cpp_helper_libs::linear_algebra::UnitVector3 &radial,
                          const cpp_helper_libs::linear_algebra::UnitVector3 &tangent) noexcept;

  cpp_helper_libs::linear_algebra::UnitVector3 radial() const noexcept { return radial_; }
  cpp_helper_libs::linear_algebra::UnitVector3 normal() const noexcept { return normal_; }
  cpp_helper_libs::linear_algebra::UnitVector3 tangent() const noexcept { return tangent_; }

  /**
   * @brief Advance along the great-circle geodesic defined by this ray.
   */
  SphericalRay project_forward(cpp_helper_libs::quantities::Angle arc_length) const noexcept;

  /**
   * @brief Exact-math variant of @ref project_forward.
   */
  SphericalRay project_forward_exact(cpp_helper_libs::quantities::Angle arc_length) const noexcept;

  /**
   * @brief Rotate the tangent/normal frame around radial.
   */
  SphericalRay rotate_about_radial(cpp_helper_libs::quantities::Angle angle) const noexcept;

  /**
   * @brief Exact-math variant of @ref rotate_about_radial.
   */
  SphericalRay rotate_about_radial_exact(cpp_helper_libs::quantities::Angle angle) const noexcept;

private:
  SphericalRay(const cpp_helper_libs::linear_algebra::UnitVector3 &radial,
               const cpp_helper_libs::linear_algebra::UnitVector3 &normal,
               const cpp_helper_libs::linear_algebra::UnitVector3 &tangent) noexcept
      : radial_(radial), normal_(normal), tangent_(tangent) {}

  /// Unit radial from the sphere origin to the surface point.
  cpp_helper_libs::linear_algebra::UnitVector3 radial_;
  /// Unit normal that defines the osculating great-circle plane orientation.
  cpp_helper_libs::linear_algebra::UnitVector3 normal_;
  /// Unit tangent that points in the forward direction on the sphere surface.
  cpp_helper_libs::linear_algebra::UnitVector3 tangent_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_RAY_HPP
