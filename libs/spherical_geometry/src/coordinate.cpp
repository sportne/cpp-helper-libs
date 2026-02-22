// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"

namespace cpp_helper_libs::spherical_geometry {
namespace {

// pi/2 in radians; latitude bounds are [-pi/2, +pi/2].
constexpr double kHalfPi = std::numbers::pi_v<double> / 2.0;
// pi in radians; used by longitude wrapping.
constexpr double kPi = std::numbers::pi_v<double>;
// Full turn in radians; used by longitude wrapping.
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Coordinate::Coordinate(cpp_helper_libs::quantities::Angle latitude,
                       cpp_helper_libs::quantities::Angle longitude) noexcept
    : latitude_(clamp_latitude(latitude)), longitude_(wrap_longitude(longitude)) {}

Coordinate Coordinate::degrees(const double latitude_degrees,
                               const double longitude_degrees) noexcept {
  return {cpp_helper_libs::quantities::Angle::degrees(latitude_degrees),
          cpp_helper_libs::quantities::Angle::degrees(longitude_degrees)};
}

Coordinate Coordinate::radians(const double latitude_radians,
                               const double longitude_radians) noexcept {
  return {cpp_helper_libs::quantities::Angle::radians(latitude_radians),
          cpp_helper_libs::quantities::Angle::radians(longitude_radians)};
}

Coordinate
Coordinate::from_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept {
  const double latitude_radians_value = std::asin(std::clamp(radial.z(), -1.0, 1.0));
  const double longitude_radians_value = std::atan2(radial.y(), radial.x());
  return {cpp_helper_libs::quantities::Angle::radians(latitude_radians_value),
          cpp_helper_libs::quantities::Angle::radians(longitude_radians_value)};
}

cpp_helper_libs::linear_algebra::UnitVector3 Coordinate::to_radial() const noexcept {
  const double latitude_radians = latitude_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double longitude_radians = longitude_.in(cpp_helper_libs::quantities::Angle::Unit::Radian);

  const double cos_latitude = std::cos(latitude_radians);
  const double x_value = cos_latitude * std::cos(longitude_radians);
  const double y_value = cos_latitude * std::sin(longitude_radians);
  const double z_value = std::sin(latitude_radians);

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> by_components =
      cpp_helper_libs::linear_algebra::UnitVector3::from_components(x_value, y_value, z_value);
  if (by_components.has_value()) {
    return by_components.value();
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> normalized =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(
          cpp_helper_libs::linear_algebra::Vector3(x_value, y_value, z_value));
  if (normalized.has_value()) {
    return normalized.value();
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> default_axis =
      cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 0.0, 1.0);
  if (default_axis.has_value()) {
    return default_axis.value();
  }

  const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> normalized_default_axis =
      cpp_helper_libs::linear_algebra::UnitVector3::from_vector(
          cpp_helper_libs::linear_algebra::Vector3(0.0, 0.0, 1.0));
  if (normalized_default_axis.has_value()) {
    return normalized_default_axis.value();
  }

  std::abort();
}

cpp_helper_libs::quantities::Angle
Coordinate::clamp_latitude(const cpp_helper_libs::quantities::Angle latitude) noexcept {
  const double latitude_radians = latitude.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double clamped = std::clamp(latitude_radians, -kHalfPi, kHalfPi);
  return cpp_helper_libs::quantities::Angle::radians(clamped);
}

cpp_helper_libs::quantities::Angle
Coordinate::wrap_longitude(const cpp_helper_libs::quantities::Angle longitude) noexcept {
  const double longitude_radians_value =
      longitude.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  double wrapped = std::fmod(longitude_radians_value + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  wrapped -= kPi;
  return cpp_helper_libs::quantities::Angle::radians(wrapped);
}

} // namespace cpp_helper_libs::spherical_geometry
