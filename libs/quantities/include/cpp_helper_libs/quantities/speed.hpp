// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_SPEED_HPP
#define CPP_HELPER_LIBS_QUANTITIES_SPEED_HPP

#include <cstddef>
#include <string_view>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical speed quantity stored canonically in meters per second.
 */
class Speed final : public QuantityBase<Speed> {
public:
  /**
   * @brief Supported speed units for construction and conversion.
   */
  enum class Unit {
    MetersPerSecond,
    KilometersPerHour,
    MilesPerHour,
    FeetPerSecond,
  };

  /**
   * @brief Construct a speed value from a magnitude and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Speed(double value, Unit unit);

  /**
   * @brief Create speed in meters per second.
   *
   * @param value Magnitude in m/s.
   * @return Speed value.
   */
  static Speed meters_per_second(double value);

  /**
   * @brief Create speed in kilometers per hour.
   *
   * @param value Magnitude in km/h.
   * @return Speed value.
   */
  static Speed kilometers_per_hour(double value);

  /**
   * @brief Create speed in miles per hour.
   *
   * @param value Magnitude in mph.
   * @return Speed value.
   */
  static Speed miles_per_hour(double value);

  /**
   * @brief Create speed in feet per second.
   *
   * @param value Magnitude in ft/s.
   * @return Speed value.
   */
  static Speed feet_per_second(double value);

  /**
   * @brief Convert this speed quantity into another unit.
   *
   * @param unit Target speed unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct a speed quantity from canonical raw meters per second.
   *
   * @param raw Canonical value in meters per second.
   * @return Speed value with canonical storage set to @p raw.
   */
  static Speed from_raw(double raw) noexcept;

private:
  explicit constexpr Speed(double raw) noexcept : QuantityBase(raw) {}

  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Speed> {
  size_t operator()(const cpp_helper_libs::quantities::Speed &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_SPEED_HPP
