// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "spherical_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <numbers>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::spherical_geometry::internal {
namespace {

// Full turn in radians for angle wrapping.
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
// Tolerance for support-surface checks (point vs support plane/circle).
constexpr double kToleranceSupport = 1e-10;
// Tolerance for normalized parameters in [0, 1].
constexpr double kToleranceParameter = 1e-10;
// Tolerance for comparing two unit radials as representing the same point.
constexpr double kToleranceRadial = 1e-10;

} // namespace

NumericPolicy tolerant_policy() noexcept {
  return {
      .support_epsilon = kToleranceSupport,
      .parameter_epsilon = kToleranceParameter,
      .radial_epsilon = kToleranceRadial,
  };
}

NumericPolicy exact_policy() noexcept {
  return {
      .support_epsilon = 0.0,
      .parameter_epsilon = 0.0,
      .radial_epsilon = 0.0,
  };
}

NumericPolicy select_policy(const bool exact) noexcept {
  return exact ? exact_policy() : tolerant_policy();
}

bool nearly_equal(const double left, const double right, const double epsilon) noexcept {
  return std::fabs(left - right) <= epsilon;
}

double wrap_zero_to_two_pi(const double angle_radians) noexcept {
  double wrapped = std::fmod(angle_radians, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

double clamp_cosine(const double value) noexcept { return std::clamp(value, -1.0, 1.0); }

bool same_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &left,
                 const cpp_helper_libs::linear_algebra::UnitVector3 &right,
                 const NumericPolicy &policy) noexcept {
  if (policy.radial_epsilon == 0.0) {
    return left == right;
  }

  const double cosine = clamp_cosine(left.dot(right));
  return (1.0 - cosine) <= policy.radial_epsilon;
}

cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_exact(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                        const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                        const double angle_radians) noexcept {
  const double cosine_value = std::cos(angle_radians);
  const double sine_value = std::sin(angle_radians);
  const cpp_helper_libs::linear_algebra::Vector3 rotated =
      value.scaled_by(cosine_value) + (axis.cross(value) * sine_value) +
      axis.scaled_by(axis.dot(value) * (1.0 - cosine_value));

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> unit_value =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(rotated);
  if (unit_value.has_value()) {
    return unit_value.value();
  }

  return value;
}

cpp_helper_libs::linear_algebra::UnitVector3
rotate_about_axis_tolerant(const cpp_helper_libs::linear_algebra::UnitVector3 &value,
                           const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                           const double angle_radians) noexcept {
  return rotate_about_axis_exact(value, axis, angle_radians);
}

std::optional<cpp_helper_libs::linear_algebra::UnitVector3>
any_orthogonal_unit(const cpp_helper_libs::linear_algebra::UnitVector3 &axis) noexcept {
  const std::array<cpp_helper_libs::linear_algebra::Vector3, 3> candidates = {
      cpp_helper_libs::linear_algebra::Vector3(1.0, 0.0, 0.0),
      cpp_helper_libs::linear_algebra::Vector3(0.0, 1.0, 0.0),
      cpp_helper_libs::linear_algebra::Vector3(0.0, 0.0, 1.0),
  };

  for (const cpp_helper_libs::linear_algebra::Vector3 &candidate : candidates) {
    const cpp_helper_libs::linear_algebra::Vector3 tangent =
        candidate - axis.scaled_by(axis.dot(candidate));
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> tangent_unit =
        tangent.normalized();
    if (tangent_unit.has_value()) {
      return tangent_unit;
    }
  }

  return std::nullopt;
}

SphericalRay make_ray_on_oriented_circle(const cpp_helper_libs::linear_algebra::UnitVector3 &axis,
                                         const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                         const double direction_sign, const bool exact) noexcept {
  (void)exact;
  const std::array<cpp_helper_libs::linear_algebra::Vector3, 4> candidate_directions = {
      axis.cross(point) * direction_sign, cpp_helper_libs::linear_algebra::Vector3(1.0, 0.0, 0.0),
      cpp_helper_libs::linear_algebra::Vector3(0.0, 1.0, 0.0),
      cpp_helper_libs::linear_algebra::Vector3(0.0, 0.0, 1.0)};

  for (const cpp_helper_libs::linear_algebra::Vector3 &candidate_direction : candidate_directions) {
    const cpp_helper_libs::linear_algebra::Vector3 tangent_projection =
        candidate_direction - point.scaled_by(point.dot(candidate_direction));
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> tangent_unit =
        tangent_projection.normalized();
    if (!tangent_unit.has_value()) {
      continue;
    }

    const std::optional<SphericalRay> ray =
        SphericalRay::from_radial_and_tangent(point, tangent_unit.value());
    if (ray.has_value()) {
      return ray.value();
    }
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> orthogonal =
      any_orthogonal_unit(point);
  if (orthogonal.has_value()) {
    const std::optional<SphericalRay> ray =
        SphericalRay::from_radial_and_tangent(point, orthogonal.value());
    if (ray.has_value()) {
      return ray.value();
    }
  }

  std::abort();
}

std::optional<CurveLocation> locate_point_on_oriented_circle(
    const cpp_helper_libs::linear_algebra::UnitVector3 &axis, const double support_constant,
    const cpp_helper_libs::linear_algebra::UnitVector3 &start_point, const double signed_sweep,
    const cpp_helper_libs::linear_algebra::UnitVector3 &point, const bool exact) noexcept {
  const NumericPolicy policy = select_policy(exact);
  const double support_error = axis.dot(point) - support_constant;
  if (std::fabs(support_error) > policy.support_epsilon) {
    return std::nullopt;
  }

  const double total_sweep = std::fabs(signed_sweep);
  if (total_sweep == 0.0) {
    if (same_radial(start_point, point, policy)) {
      return CurveLocation{.parameter = 0.0, .at_start = true, .at_end = true};
    }
    return std::nullopt;
  }

  const double signed_angle =
      std::atan2(axis.dot(start_point.cross(point)), clamp_cosine(start_point.dot(point)));

  const double travel = (signed_sweep >= 0.0) ? wrap_zero_to_two_pi(signed_angle)
                                              : wrap_zero_to_two_pi(-signed_angle);

  if (exact) {
    if (travel < 0.0 || travel > total_sweep) {
      return std::nullopt;
    }
  } else {
    if (travel < -policy.parameter_epsilon || travel > (total_sweep + policy.parameter_epsilon)) {
      return std::nullopt;
    }
  }

  const double bounded_travel = std::clamp(travel, 0.0, total_sweep);
  const double parameter = bounded_travel / total_sweep;

  const bool at_start = nearly_equal(bounded_travel, 0.0, policy.parameter_epsilon);
  const bool at_end = nearly_equal(bounded_travel, total_sweep, policy.parameter_epsilon);

  return CurveLocation{.parameter = parameter, .at_start = at_start, .at_end = at_end};
}

double small_arc_length_radians(const double radius_radians, const double sweep_radians) noexcept {
  return std::fabs(std::sin(radius_radians) * sweep_radians);
}

} // namespace cpp_helper_libs::spherical_geometry::internal
