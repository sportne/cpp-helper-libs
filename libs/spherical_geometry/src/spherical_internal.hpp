// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP

#include <optional>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/intersection.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry {
class SphericalCurve;
} // namespace cpp_helper_libs::spherical_geometry

namespace cpp_helper_libs::spherical_geometry::internal {

/**
 * @brief Numeric tolerances used by tolerant/exact geometric predicates.
 *
 * All spherical-geometry algorithms choose one policy at call time:
 * - tolerant policy: robust to floating-point roundoff
 * - exact policy: strict comparisons (all epsilons set to zero)
 */
struct NumericPolicy final {
  /// Allowed error for checking if a point lies on a support circle/plane.
  double support_epsilon;
  /// Allowed error for normalized curve parameters and sweep bounds.
  double parameter_epsilon;
  /// Allowed radial mismatch when comparing two unit vectors as "same point".
  double radial_epsilon;
};

/**
 * @brief Return tolerant numeric thresholds for geometric operations.
 */
NumericPolicy tolerant_policy() noexcept;
/**
 * @brief Return exact numeric thresholds (all zero).
 */
NumericPolicy exact_policy() noexcept;
/**
 * @brief Select tolerant or exact policy based on caller intent.
 *
 * @param exact `true` for strict comparisons, `false` for tolerant comparisons.
 */
NumericPolicy select_policy(bool exact) noexcept;

/**
 * @brief Compare two scalars with absolute tolerance.
 *
 * @param left First value.
 * @param right Second value.
 * @param epsilon Allowed absolute error.
 */
bool nearly_equal(double left, double right, double epsilon) noexcept;

/**
 * @brief Wrap an angle into [0, 2*pi).
 */
double wrap_zero_to_two_pi(double angle_radians) noexcept;

/**
 * @brief Clamp a cosine-like value into [-1, +1] before inverse trig.
 */
double clamp_cosine(double value) noexcept;

/**
 * @brief Compare whether two unit radials represent the same spherical point.
 *
 * Contract:
 * - exact policy uses component-wise equality
 * - tolerant policy uses angular closeness via dot product
 */
bool same_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &left,
                 const cpp_helper_libs::linear_algebra::UnitVector3 &right,
                 const NumericPolicy &policy) noexcept;

/**
 * @brief Rotate a unit radial around an axis using Rodrigues' rotation formula.
 *
 * Assumptions:
 * - @p axis is a unit vector.
 * - @p value is a unit vector.
 */
cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_exact(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                        const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                        double angle_radians) noexcept;

/**
 * @brief Tolerant variant of @ref rotate_about_axis_exact.
 *
 * Today this uses the same implementation as exact rotation, but the API
 * intentionally separates policy-specific behavior for future tuning.
 */
cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_tolerant(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                           const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                           double angle_radians) noexcept;

/**
 * @brief Produce any valid unit vector orthogonal to @p axis.
 *
 * @return Orthogonal unit vector, or `std::nullopt` when construction fails.
 */
std::optional<cpp_helper_libs::linear_algebra::UnitVector3>
any_orthogonal_unit(const cpp_helper_libs::linear_algebra::UnitVector3 &axis) noexcept;

/**
 * @brief Build a `SphericalRay` tangent to an oriented support circle at @p point.
 *
 * Contract:
 * - `radial == point`
 * - tangent direction follows @p direction_sign around @p axis
 *
 * Assumptions:
 * - @p point lies on the unit sphere.
 * - @p axis is the support axis of the curve's circle.
 */
SphericalRay make_ray_on_oriented_circle(const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                                         const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                         double direction_sign, bool exact) noexcept;

/**
 * @brief Locate a point on an oriented spherical-circle sweep.
 *
 * Contract:
 * - returns `std::nullopt` when point is not on the support circle or outside the sweep
 * - otherwise returns normalized parameter in [0, 1] plus endpoint flags
 *
 * Assumptions:
 * - @p start_point lies on the support circle defined by (@p axis, @p support_constant).
 * - @p signed_sweep orientation matches the intended traversal direction.
 */
std::optional<CurveLocation> locate_point_on_oriented_circle(
    const cpp_helper_libs::linear_algebra::UnitVector3 &axis, double support_constant,
    const cpp_helper_libs::linear_algebra::UnitVector3 &start_point, double signed_sweep,
    const cpp_helper_libs::linear_algebra::UnitVector3 &point, bool exact) noexcept;

/**
 * @brief Compute geometric length of a small-circle arc.
 *
 * Formula: `length = |sin(radius) * sweep|`.
 */
double small_arc_length_radians(double radius_radians, double sweep_radians) noexcept;

/**
 * @brief Boolean intersection query that short-circuits without materializing records.
 *
 * @param first First curve.
 * @param second Second curve.
 * @param exact `true` for exact comparisons, `false` for tolerant policy.
 * @param include_endpoint_touches When `false`, endpoint-only touches are ignored.
 */
bool curves_intersect(const SphericalCurve &first, const SphericalCurve &second, bool exact,
                      bool include_endpoint_touches) noexcept;

} // namespace cpp_helper_libs::spherical_geometry::internal

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_SPHERICAL_INTERNAL_HPP
