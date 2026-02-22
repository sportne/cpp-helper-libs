// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "spherical_internal.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

struct SupportIntersection final {
  // True when both curves lie on the same supporting circle.
  bool coincident;
  // Candidate geometric intersection points between two support circles.
  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> points;
};

void add_unique_parameter(std::vector<double> *parameters, const double value,
                          const internal::NumericPolicy &policy) {
  const bool already_present =
      std::any_of(parameters->begin(), parameters->end(), [&](const double existing) {
        return internal::nearly_equal(existing, value, policy.parameter_epsilon);
      });
  if (already_present) {
    return;
  }

  parameters->push_back(std::clamp(value, 0.0, 1.0));
}

bool parameter_within_interval(const double value, const double interval_start,
                               const double interval_end, const internal::NumericPolicy &policy) {
  return value >= (interval_start - policy.parameter_epsilon) &&
         value <= (interval_end + policy.parameter_epsilon);
}

void add_unique_point(std::vector<cpp_helper_libs::linear_algebra::UnitVector3> *points,
                      const cpp_helper_libs::linear_algebra::UnitVector3 &candidate,
                      const internal::NumericPolicy &policy) {
  const bool already_present =
      std::any_of(points->begin(), points->end(),
                  [&](const cpp_helper_libs::linear_algebra::UnitVector3 &existing) {
                    return internal::same_radial(existing, candidate, policy);
                  });
  if (already_present) {
    return;
  }
  points->push_back(candidate);
}

SupportIntersection intersect_supports(const SphericalCurve &first, const SphericalCurve &second,
                                       const internal::NumericPolicy &policy) {
  const cpp_helper_libs::linear_algebra::UnitVector3 axis_first = first.support_axis();
  const cpp_helper_libs::linear_algebra::UnitVector3 axis_second = second.support_axis();
  const double support_first = first.support_constant();
  const double support_second = second.support_constant();

  const cpp_helper_libs::linear_algebra::Vector3 axis_cross = axis_first.cross(axis_second);
  const double cross_squared_magnitude = axis_cross.squared_magnitude();

  if (cross_squared_magnitude <= (policy.support_epsilon * policy.support_epsilon)) {
    const double alignment = axis_first.dot(axis_second);
    if (std::fabs(std::fabs(alignment) - 1.0) > policy.support_epsilon) {
      return {.coincident = false, .points = {}};
    }

    const bool same_support =
        (alignment >= 0.0) ? (std::fabs(support_first - support_second) <= policy.support_epsilon)
                           : (std::fabs(support_first + support_second) <= policy.support_epsilon);
    return {.coincident = same_support, .points = {}};
  }

  const cpp_helper_libs::linear_algebra::Vector3 intersection_base =
      ((axis_second.cross(axis_cross) * support_first) +
       (axis_cross.cross(axis_first.as_vector()) * support_second)) *
      (1.0 / cross_squared_magnitude);

  const double base_squared = intersection_base.squared_magnitude();
  const double discriminant = 1.0 - base_squared;
  if (discriminant < -policy.support_epsilon) {
    return {.coincident = false, .points = {}};
  }

  const double bounded_discriminant = std::max(0.0, discriminant);
  const double lambda = std::sqrt(bounded_discriminant / cross_squared_magnitude);

  std::vector<cpp_helper_libs::linear_algebra::UnitVector3> points;
  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> first_point =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(intersection_base +
                                                                (axis_cross * lambda));
  if (first_point.has_value()) {
    add_unique_point(&points, first_point.value(), policy);
  }

  if (lambda > policy.support_epsilon) {
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> second_point =
        cpp_helper_libs::linear_algebra::UnitVector3::from_vector(intersection_base -
                                                                  (axis_cross * lambda));
    if (second_point.has_value()) {
      add_unique_point(&points, second_point.value(), policy);
    }
  }

  return {.coincident = false, .points = points};
}

CurveIntersection
point_intersection_for_locations(const cpp_helper_libs::linear_algebra::UnitVector3 &point,
                                 const CurveLocation &first_location,
                                 const CurveLocation &second_location) {
  if (first_location.at_start || first_location.at_end || second_location.at_start ||
      second_location.at_end) {
    return CurveIntersection::endpoint_touch(point, first_location, second_location);
  }

  return CurveIntersection::point(point, first_location, second_location);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::vector<CurveIntersection> coincident_intersections(const SphericalCurve &first,
                                                        const SphericalCurve &second,
                                                        const bool exact) {
  const internal::NumericPolicy policy = internal::select_policy(exact);

  std::vector<double> cuts{0.0, 1.0};

  const std::optional<CurveLocation> second_start_on_first =
      first.locate_point(second.start_radial(), exact);
  if (second_start_on_first.has_value()) {
    add_unique_parameter(&cuts, second_start_on_first->parameter, policy);
  }

  const std::optional<CurveLocation> second_end_on_first =
      first.locate_point(second.end_radial(), exact);
  if (second_end_on_first.has_value()) {
    add_unique_parameter(&cuts, second_end_on_first->parameter, policy);
  }

  std::sort(cuts.begin(), cuts.end());

  std::vector<std::pair<double, double>> overlap_intervals;
  for (std::size_t index = 0U; index + 1U < cuts.size(); ++index) {
    const double start_parameter = cuts[index];
    const double end_parameter = cuts[index + 1U];
    if ((end_parameter - start_parameter) <= policy.parameter_epsilon) {
      continue;
    }

    const double midpoint = 0.5 * (start_parameter + end_parameter);
    const cpp_helper_libs::linear_algebra::UnitVector3 sample_point =
        first.point_at_parameter(midpoint);

    if (second.locate_point(sample_point, exact).has_value()) {
      overlap_intervals.emplace_back(start_parameter, end_parameter);
    }
  }

  std::vector<CurveIntersection> intersections;

  for (const std::pair<double, double> &overlap_interval : overlap_intervals) {
    const cpp_helper_libs::linear_algebra::UnitVector3 first_point =
        first.point_at_parameter(overlap_interval.first);
    const cpp_helper_libs::linear_algebra::UnitVector3 second_point =
        first.point_at_parameter(overlap_interval.second);

    const std::optional<CurveLocation> first_first_location =
        first.locate_point(first_point, exact);
    const std::optional<CurveLocation> first_second_location =
        first.locate_point(second_point, exact);
    const std::optional<CurveLocation> second_first_location =
        second.locate_point(first_point, exact);
    const std::optional<CurveLocation> second_second_location =
        second.locate_point(second_point, exact);

    if (!first_first_location.has_value() || !first_second_location.has_value() ||
        !second_first_location.has_value() || !second_second_location.has_value()) {
      continue;
    }

    if (internal::same_radial(first_point, second_point, policy)) {
      intersections.push_back(point_intersection_for_locations(
          first_point, first_first_location.value(), second_first_location.value()));
      continue;
    }

    intersections.push_back(CurveIntersection::overlap_segment(
        first_point, second_point, first_first_location.value(), first_second_location.value(),
        second_first_location.value(), second_second_location.value()));
  }

  for (const double cut : cuts) {
    const bool in_overlap =
        std::any_of(overlap_intervals.begin(), overlap_intervals.end(),
                    [&](const std::pair<double, double> &overlap_interval) {
                      return parameter_within_interval(cut, overlap_interval.first,
                                                       overlap_interval.second, policy);
                    });
    if (in_overlap) {
      continue;
    }

    const cpp_helper_libs::linear_algebra::UnitVector3 point = first.point_at_parameter(cut);
    const std::optional<CurveLocation> first_location = first.locate_point(point, exact);
    const std::optional<CurveLocation> second_location = second.locate_point(point, exact);

    if (!first_location.has_value() || !second_location.has_value()) {
      continue;
    }

    bool duplicate = false;
    for (const CurveIntersection &existing : intersections) {
      if (existing.kind() == CurveIntersectionKind::OverlapSegment) {
        continue;
      }
      if (internal::same_radial(existing.first_point(), point, policy)) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }

    intersections.push_back(
        point_intersection_for_locations(point, first_location.value(), second_location.value()));
  }

  return intersections;
}

CurveLocation zero_length_location() {
  return CurveLocation{.parameter = 0.0, .at_start = true, .at_end = true};
}

} // namespace

std::vector<CurveIntersection>
SphericalCurve::intersections_with(const SphericalCurve &other) const noexcept {
  return intersections_with_policy(other, false);
}

std::vector<CurveIntersection>
SphericalCurve::intersections_with_exact(const SphericalCurve &other) const noexcept {
  return intersections_with_policy(other, true);
}

std::vector<CurveIntersection>
SphericalCurve::intersections_with_policy(const SphericalCurve &other,
                                          const bool exact) const noexcept {
  const internal::NumericPolicy policy = internal::select_policy(exact);

  if (is_zero_length_curve() && other.is_zero_length_curve()) {
    if (internal::same_radial(start_radial(), other.start_radial(), policy)) {
      return {CurveIntersection::endpoint_touch(start_radial(), zero_length_location(),
                                                zero_length_location())};
    }
    return {};
  }

  if (is_zero_length_curve()) {
    const std::optional<CurveLocation> second_location = other.locate_point(start_radial(), exact);
    if (!second_location.has_value()) {
      return {};
    }

    return {point_intersection_for_locations(start_radial(), zero_length_location(),
                                             second_location.value())};
  }

  if (other.is_zero_length_curve()) {
    const std::optional<CurveLocation> first_location = locate_point(other.start_radial(), exact);
    if (!first_location.has_value()) {
      return {};
    }

    return {point_intersection_for_locations(other.start_radial(), first_location.value(),
                                             zero_length_location())};
  }

  const SupportIntersection support_intersection = intersect_supports(*this, other, policy);

  if (support_intersection.coincident) {
    return coincident_intersections(*this, other, exact);
  }

  std::vector<CurveIntersection> intersections;
  for (const cpp_helper_libs::linear_algebra::UnitVector3 &candidate :
       support_intersection.points) {
    const std::optional<CurveLocation> first_location = locate_point(candidate, exact);
    const std::optional<CurveLocation> second_location = other.locate_point(candidate, exact);
    if (!first_location.has_value() || !second_location.has_value()) {
      continue;
    }
    const CurveLocation first_location_value = first_location.value();
    const CurveLocation second_location_value = second_location.value();

    const bool duplicate = std::any_of(
        intersections.begin(), intersections.end(), [&](const CurveIntersection &existing) {
          return internal::same_radial(existing.first_point(), candidate, policy);
        });
    if (duplicate) {
      continue;
    }

    intersections.push_back(
        point_intersection_for_locations(candidate, first_location_value, second_location_value));
  }

  return intersections;
}

} // namespace cpp_helper_libs::spherical_geometry
