// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_GREAT_CIRCLE_ARC_COMMON_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_GREAT_CIRCLE_ARC_COMMON_HPP

#include <algorithm>
#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry::internal {

inline cpp_helper_libs::linear_algebra::UnitVector3
great_circle_start_radial(const SphericalRay &start_ray) noexcept {
  return start_ray.radial();
}

inline cpp_helper_libs::linear_algebra::UnitVector3
great_circle_end_radial(const SphericalRay &start_ray,
                        const cpp_helper_libs::quantities::Angle sweep) noexcept {
  return start_ray.project_forward(sweep).radial();
}

inline SphericalRay great_circle_end_ray(const SphericalRay &start_ray,
                                         const cpp_helper_libs::quantities::Angle sweep) noexcept {
  return start_ray.project_forward(sweep);
}

inline cpp_helper_libs::linear_algebra::UnitVector3
great_circle_support_axis(const SphericalRay &start_ray) noexcept {
  return start_ray.normal();
}

inline double
great_circle_signed_sweep_radians(const cpp_helper_libs::quantities::Angle sweep) noexcept {
  return sweep.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
}

inline std::optional<CurveLocation> great_circle_locate_point(
    const SphericalRay &start_ray, const cpp_helper_libs::quantities::Angle sweep,
    const cpp_helper_libs::linear_algebra::UnitVector3 &point, const bool exact) noexcept {
  return locate_point_on_oriented_circle(great_circle_support_axis(start_ray), 0.0,
                                         great_circle_start_radial(start_ray),
                                         great_circle_signed_sweep_radians(sweep), point, exact);
}

inline cpp_helper_libs::linear_algebra::UnitVector3
great_circle_point_at_parameter(const SphericalRay &start_ray,
                                const cpp_helper_libs::quantities::Angle sweep,
                                const double parameter) noexcept {
  const double clamped = std::clamp(parameter, 0.0, 1.0);
  return rotate_about_axis_tolerant(great_circle_start_radial(start_ray),
                                    great_circle_support_axis(start_ray),
                                    clamped * great_circle_signed_sweep_radians(sweep));
}

} // namespace cpp_helper_libs::spherical_geometry::internal

#define CPPHL_DEFINE_GREAT_CIRCLE_ARC_CORE_METHODS(ArcType, StartRayMember, SweepMember)           \
  cpp_helper_libs::linear_algebra::UnitVector3 ArcType::start_radial() const noexcept {            \
    return internal::great_circle_start_radial(StartRayMember);                                    \
  }                                                                                                \
                                                                                                   \
  cpp_helper_libs::linear_algebra::UnitVector3 ArcType::end_radial() const noexcept {              \
    return internal::great_circle_end_radial(StartRayMember, SweepMember);                         \
  }                                                                                                \
                                                                                                   \
  SphericalRay ArcType::start_ray() const noexcept { return StartRayMember; }                      \
                                                                                                   \
  SphericalRay ArcType::end_ray() const noexcept {                                                 \
    return internal::great_circle_end_ray(StartRayMember, SweepMember);                            \
  }                                                                                                \
                                                                                                   \
  cpp_helper_libs::quantities::Angle ArcType::length() const noexcept { return SweepMember; }      \
                                                                                                   \
  cpp_helper_libs::linear_algebra::UnitVector3 ArcType::support_axis() const noexcept {            \
    return internal::great_circle_support_axis(StartRayMember);                                    \
  }                                                                                                \
                                                                                                   \
  double ArcType::support_constant() const noexcept { return 0.0; }                                \
                                                                                                   \
  double ArcType::signed_sweep_radians() const noexcept {                                          \
    return internal::great_circle_signed_sweep_radians(SweepMember);                               \
  }                                                                                                \
                                                                                                   \
  std::optional<CurveLocation> ArcType::locate_point(                                              \
      const cpp_helper_libs::linear_algebra::UnitVector3 &point, const bool exact)                 \
      const noexcept {                                                                             \
    return internal::great_circle_locate_point(StartRayMember, SweepMember, point, exact);         \
  }                                                                                                \
                                                                                                   \
  cpp_helper_libs::linear_algebra::UnitVector3 ArcType::point_at_parameter(const double parameter) \
      const noexcept {                                                                             \
    return internal::great_circle_point_at_parameter(StartRayMember, SweepMember, parameter);      \
  }                                                                                                \
                                                                                                   \
  bool ArcType::is_zero_length_curve() const noexcept { return false; }

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_GREAT_CIRCLE_ARC_COMMON_HPP
