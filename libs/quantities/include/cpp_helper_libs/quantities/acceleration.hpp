// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP
#define CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical acceleration quantity stored canonically in meters per second squared.
 */
class Acceleration final : public QuantityBase<Acceleration> {
public:
  /**
   * @brief Supported acceleration units for construction and conversion.
   */
  enum class Unit {
    MetersPerSecondSquared,
    FeetPerSecondSquared,
    StandardGravity,
  };

  /**
   * @brief Construct an acceleration value from a magnitude and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Acceleration(double value, Unit unit);

  /**
   * @brief Create acceleration in meters per second squared.
   *
   * @param value Magnitude in m/s^2.
   * @return Acceleration value.
   */
  static Acceleration meters_per_second_squared(double value);

  /**
   * @brief Create acceleration in feet per second squared.
   *
   * @param value Magnitude in ft/s^2.
   * @return Acceleration value.
   */
  static Acceleration feet_per_second_squared(double value);

  /**
   * @brief Create acceleration in standard gravity units.
   *
   * @param value Magnitude in g.
   * @return Acceleration value.
   */
  static Acceleration standard_gravity(double value);

  /**
   * @brief Convert this acceleration quantity into another unit.
   *
   * @param unit Target acceleration unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct acceleration from canonical raw meters per second squared.
   *
   * @param raw Canonical value in m/s^2.
   * @return Acceleration value with canonical storage set to @p raw.
   */
  static Acceleration from_raw(double raw) noexcept;

private:
  explicit constexpr Acceleration(double raw) noexcept : QuantityBase(raw) {}

  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Acceleration> {
  size_t operator()(const cpp_helper_libs::quantities::Acceleration &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP
