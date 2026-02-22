// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_LENGTH_HPP
#define CPP_HELPER_LIBS_QUANTITIES_LENGTH_HPP

#include <cstddef>
#include <string_view>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical length quantity stored canonically in meters.
 */
class Length final : public QuantityBase<Length> {
public:
  /**
   * @brief Supported length units for construction and conversion.
   */
  enum class Unit {
    Millimeter,
    Centimeter,
    Meter,
    Kilometer,
    Inch,
    Foot,
    Yard,
    Mile,
  };

  /**
   * @brief Construct a length from a value and unit.
   *
   * @param value Numeric magnitude in the supplied unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Length(double value, Unit unit);

  /**
   * @brief Create a length in millimeters.
   *
   * @param value Magnitude in millimeters.
   * @return Length value.
   */
  static Length millimeters(double value);

  /**
   * @brief Create a length in centimeters.
   *
   * @param value Magnitude in centimeters.
   * @return Length value.
   */
  static Length centimeters(double value);

  /**
   * @brief Create a length in meters.
   *
   * @param value Magnitude in meters.
   * @return Length value.
   */
  static Length meters(double value);

  /**
   * @brief Create a length in kilometers.
   *
   * @param value Magnitude in kilometers.
   * @return Length value.
   */
  static Length kilometers(double value);

  /**
   * @brief Create a length in inches.
   *
   * @param value Magnitude in inches.
   * @return Length value.
   */
  static Length inches(double value);

  /**
   * @brief Create a length in feet.
   *
   * @param value Magnitude in feet.
   * @return Length value.
   */
  static Length feet(double value);

  /**
   * @brief Create a length in yards.
   *
   * @param value Magnitude in yards.
   * @return Length value.
   */
  static Length yards(double value);

  /**
   * @brief Create a length in miles.
   *
   * @param value Magnitude in miles.
   * @return Length value.
   */
  static Length miles(double value);

  /**
   * @brief Convert this length into the requested unit.
   *
   * @param unit Target unit.
   * @return Length magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct a length directly from canonical raw meters.
   *
   * @param raw Canonical length in meters.
   * @return Length value with raw storage initialized to @p raw.
   */
  static Length from_raw(double raw) noexcept;

private:
  /// Construct directly from canonical raw meters.
  explicit constexpr Length(double raw) noexcept : QuantityBase(raw) {}

  /// Convert from a user-facing unit into canonical raw meters.
  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Length> {
  size_t operator()(const cpp_helper_libs::quantities::Length &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_LENGTH_HPP
