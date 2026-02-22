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
  CounterClockwise,
  Clockwise,
};

/**
 * @brief Small-circle arc segment.
 */
class SmallArc final : public SphericalCurve {
public:
  static std::optional<SmallArc>
  from_center_start_direction_and_sweep(const cpp_helper_libs::linear_algebra::UnitVector3 &center,
                                        cpp_helper_libs::quantities::Angle radius,
                                        const SphericalRay &start_ray, TurnDirection turn_direction,
                                        cpp_helper_libs::quantities::Angle sweep) noexcept;

  cpp_helper_libs::linear_algebra::UnitVector3 center() const noexcept { return center_; }
  cpp_helper_libs::quantities::Angle radius() const noexcept { return radius_; }
  TurnDirection turn_direction() const noexcept { return turn_direction_; }

  cpp_helper_libs::linear_algebra::UnitVector3 start_radial() const noexcept override;
  cpp_helper_libs::linear_algebra::UnitVector3 end_radial() const noexcept override;

  SphericalRay start_ray() const noexcept override;
  SphericalRay end_ray() const noexcept override;

  cpp_helper_libs::quantities::Angle length() const noexcept override;

protected:
  cpp_helper_libs::linear_algebra::UnitVector3 support_axis() const noexcept override;
  double support_constant() const noexcept override;
  double signed_sweep_radians() const noexcept override;
  std::optional<CurveLocation>
  locate_point(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
               bool exact) const noexcept override;
  cpp_helper_libs::linear_algebra::UnitVector3
  point_at_parameter(double parameter) const noexcept override;
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
