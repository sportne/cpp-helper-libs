// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry::internal {

struct NumericPolicy final {
  /// Allowed error for checking if a point lies on a support circle/plane.
  double support_epsilon;
  /// Allowed error for normalized curve parameters and sweep bounds.
  double parameter_epsilon;
  /// Allowed radial mismatch when comparing two unit vectors as "same point".
  double radial_epsilon;
};

NumericPolicy tolerant_policy() noexcept;
NumericPolicy exact_policy() noexcept;
NumericPolicy select_policy(bool exact) noexcept;

bool nearly_equal(double left, double right, double epsilon) noexcept;

double wrap_zero_to_two_pi(double angle_radians) noexcept;

double clamp_cosine(double value) noexcept;

bool same_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &left,
                 const cpp_helper_libs::linear_algebra::UnitVector3 &right,
                 const NumericPolicy &policy) noexcept;

cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_exact(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                        const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                        double angle_radians) noexcept;

cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_tolerant(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                           const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                           double angle_radians) noexcept;

std::optional<cpp_helper_libs::linear_algebra::UnitVector3>
any_orthogonal_unit(const cpp_helper_libs::linear_algebra::UnitVector3 &axis) noexcept;

SphericalRay make_ray_on_oriented_circle(const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                                         const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                         double direction_sign, bool exact) noexcept;

std::optional<CurveLocation> locate_point_on_oriented_circle(
    const cpp_helper_libs::linear_algebra::UnitVector3 &axis, double support_constant,
    const cpp_helper_libs::linear_algebra::UnitVector3 &start_point, double signed_sweep,
    const cpp_helper_libs::linear_algebra::UnitVector3 &point, bool exact) noexcept;

double small_arc_length_radians(double radius_radians, double sweep_radians) noexcept;

} // namespace cpp_helper_libs::spherical_geometry::internal

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP
