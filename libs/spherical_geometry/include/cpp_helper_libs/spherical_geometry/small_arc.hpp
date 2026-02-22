// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SMALL_ARC_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SMALL_ARC_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Direction around the small-circle center axis.
 */
enum class TurnDirection {
  /// Positive orientation around the center axis.
  CounterClockwise,
  /// Negative orientation around the center axis.
  Clockwise,
};

/**
 * @brief Small-circle arc segment.
 */
class SmallArc final : public SphericalCurve {
public:
  /**
   * @brief Create a directed small-circle arc segment.
   *
   * @param center Unit radial to the small-circle center direction.
   * @param radius Central-angle radius from @p center to boundary points.
   * @param start_ray Start frame at boundary point; tangent must match direction.
   * @param turn_direction Traversal orientation around the center axis.
   * @param sweep Unsigned travel amount along the small circle.
   * @return Arc instance on valid geometry, otherwise `std::nullopt`.
   */
  static std::optional<SmallArc>
  from_center_start_direction_and_sweep(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                                        cpp_helper_libs::quantities::Angle radius,
                                        const SphericalRay &start_ray, TurnDirection turn_direction,
                                        cpp_helper_libs::quantities::Angle sweep) noexcept;

  /**
   * @brief Center direction of the supporting small circle.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 center() const noexcept { return center_; }
  /**
   * @brief Radius of the supporting small circle (central angle).
   */
  cpp_helper_libs::quantities::Angle radius() const noexcept { return radius_; }
  /**
   * @brief Traversal orientation around the center axis.
   */
  TurnDirection turn_direction() const noexcept { return turn_direction_; }

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
   * @brief Surface length of the arc on the unit sphere.
   */
  cpp_helper_libs::quantities::Angle length() const noexcept override;

protected:
  /**
   * @brief Support axis of the small-circle plane (`center` direction).
   */
  cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept override;
  /**
   * @brief Support-plane constant `axis.dot(point)`.
   *
   * For a small circle this equals `cos(radius)`.
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
   * Always `false` for @ref SmallArc.
   */
  bool is_zero_length_curve() const noexcept override;

private:
  SmallArc(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
           cpp_helper_libs::quantities::Angle radius, const SphericalRay &start_ray,
           TurnDirection turn_direction, cpp_helper_libs::quantities::Angle sweep) noexcept
      : center_(center), radius_(radius), start_ray_(start_ray), turn_direction_(turn_direction),
        sweep_(sweep) {}

  /// Unit radial that points to the small-circle center on the sphere.
  cpp_helper_libs::linear_algebra::UnitVector3 center_;
  /// Small-circle radius as a central angle from @ref center_ to any boundary point.
  cpp_helper_libs::quantities::Angle radius_;
  /// Oriented frame at the start point of the arc.
  SphericalRay start_ray_;
  /// Travel direction around the center axis.
  TurnDirection turn_direction_;
  /// Magnitude of travel around the small circle.
  cpp_helper_libs::quantities::Angle sweep_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SMALL_ARC_HPP
