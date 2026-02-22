// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_COORDINATE_HPP
#define CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_COORDINATE_HPP

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"

namespace cpp_helper_libs::spherical_geometry {

/**
 * @brief Geographic coordinate represented by latitude/longitude.
 *
 * Latitude is clamped to [-90, +90] degrees and longitude is wrapped to
 * [-180, +180) degrees on construction.
 */
class Coordinate final {
public:
  /**
   * @brief Construct from latitude and longitude angles.
   */
  Coordinate(cpp_helper_libs::quantities::Angle latitude,
             cpp_helper_libs::quantities::Angle longitude) noexcept;

  /**
   * @brief Construct from degree inputs.
   */
  static Coordinate degrees(double latitude_degrees, double longitude_degrees) noexcept;

  /**
   * @brief Construct from radian inputs.
   */
  static Coordinate radians(double latitude_radians, double longitude_radians) noexcept;

  /**
   * @brief Convert from a unit radial vector.
   */
  static Coordinate
  from_radial(const cpp_helper_libs::linear_algebra::UnitVector3 &radial) noexcept;

  /**
   * @brief Access normalized latitude.
   */
  cpp_helper_libs::quantities::Angle latitude() const noexcept { return latitude_; }

  /**
   * @brief Access normalized longitude.
   */
  cpp_helper_libs::quantities::Angle longitude() const noexcept { return longitude_; }

  /**
   * @brief Convert to unit radial vector on the unit sphere.
   */
  cpp_helper_libs::linear_algebra::UnitVector3 to_radial() const noexcept;

private:
  /**
   * @brief Clamp latitude to the physically meaningful range on a sphere.
   */
  static cpp_helper_libs::quantities::Angle
  clamp_latitude(cpp_helper_libs::quantities::Angle latitude) noexcept;
  /**
   * @brief Wrap longitude into the canonical interval [-180, +180) degrees.
   */
  static cpp_helper_libs::quantities::Angle
  wrap_longitude(cpp_helper_libs::quantities::Angle longitude) noexcept;

  /// Stored latitude after clamping to [-90, +90] degrees.
  cpp_helper_libs::quantities::Angle latitude_;
  /// Stored longitude after wrapping to [-180, +180) degrees.
  cpp_helper_libs::quantities::Angle longitude_;
};

} // namespace cpp_helper_libs::spherical_geometry

#endif // CPP_HELPER_LIBS_SPHERICAL_GEOMETRY_COORDINATE_HPP
