// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_MINOR_ARC_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_MINOR_ARC_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Great-circle arc segment with sweep in (0, pi).
 */
class MinorArc final : public SphericalCurve {
public:
  /**
   * @brief Create a minor arc from an oriented start ray and angular sweep.
   *
   * @param start_ray Oriented start frame; tangent direction sets traversal orientation.
   * @param sweep Central-angle sweep on the unit sphere.
   * @return Arc instance when `sweep` is in `(0, pi)`, otherwise `std::nullopt`.
   */
  static std::optional<MinorArc>
  from_start_and_sweep(const SphericalRay &start_ray,
                       cpp_helper_libs::quantities::Angle sweep) noexcept;

  /**
   * @brief Create the unique minor great-circle arc between two endpoints.
   *
   * @param start Start endpoint radial.
   * @param end End endpoint radial.
   * @return Arc instance when endpoints define a valid minor arc, else `std::nullopt`.
   */
  static std::optional<MinorArc>
  from_endpoints(const cpp_helper_libs::linear_algebra::UnitVector3 &start,
                 const cpp_helper_libs::linear_algebra::UnitVector3 &end) noexcept;

  /**
   * @brief Unit radial of the start endpoint.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept override;
  /**
   * @brief Unit radial of the end endpoint.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept override;

  /**
   * @brief Oriented frame at the start endpoint.
   */
  SphericalRay start_ray() const noexcept override;
  /**
   * @brief Oriented frame at the end endpoint.
   */
  SphericalRay end_ray() const noexcept override;

  /**
   * @brief Surface length of the arc as a central angle.
   */
  cpp_helper_libs::quantities::Angle length() const noexcept override;

protected:
  /**
   * @brief Great-circle support axis (`normal` of support plane).
   */
  cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept override;
  /**
   * @brief Great-circle support constant.
   *
   * Minor arcs lie on great circles, so this is always `0`.
   */
  double support_constant() const noexcept override;
  /**
   * @brief Signed sweep used for parameterization and intersection internals.
   */
  double signed_sweep_radians() const noexcept override;
  /**
   * @brief Locate a point on this arc and return normalized curve parameter metadata.
   */
  std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept override;
  /**
   * @brief Evaluate the arc at normalized parameter `t` in [0, 1].
   */
  cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept override;
  /**
   * @brief Report whether this curve is degenerate.
   *
   * Always `false` for @ref MinorArc.
   */
  bool is_zero_length_curve() const noexcept override;

private:
  MinorArc(const SphericalRay &start_ray, cpp_helper_libs::quantities::Angle sweep) noexcept
      : start_ray_(start_ray), sweep_(sweep) {}

  /// Oriented frame at the arc start; defines start point and geodesic orientation.
  SphericalRay start_ray_;
  /// Angular sweep along the great circle, constrained to (0, pi).
  cpp_helper_libs::quantities::Angle sweep_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_MINOR_ARC_HPP
