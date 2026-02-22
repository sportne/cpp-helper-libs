// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_TIME_HPP
#define CPP_HELPER_LIBS_QUANTITIES_TIME_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical time quantity stored canonically in seconds.
 */
class Time final : public QuantityBase<Time> {
public:
  /**
   * @brief Supported time units for construction and conversion.
   */
  enum class Unit {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Hour,
  };

  /**
   * @brief Construct a time value from a magnitude and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Time(double value, Unit unit);

  /**
   * @brief Create a time in nanoseconds.
   *
   * @param value Magnitude in nanoseconds.
   * @return Time value.
   */
  static Time nanoseconds(double value);

  /**
   * @brief Create a time in microseconds.
   *
   * @param value Magnitude in microseconds.
   * @return Time value.
   */
  static Time microseconds(double value);

  /**
   * @brief Create a time in milliseconds.
   *
   * @param value Magnitude in milliseconds.
   * @return Time value.
   */
  static Time milliseconds(double value);

  /**
   * @brief Create a time in seconds.
   *
   * @param value Magnitude in seconds.
   * @return Time value.
   */
  static Time seconds(double value);

  /**
   * @brief Create a time in minutes.
   *
   * @param value Magnitude in minutes.
   * @return Time value.
   */
  static Time minutes(double value);

  /**
   * @brief Create a time in hours.
   *
   * @param value Magnitude in hours.
   * @return Time value.
   */
  static Time hours(double value);

  /**
   * @brief Convert this time quantity into another unit.
   *
   * @param unit Target time unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct a time quantity from canonical raw seconds.
   *
   * @param raw Canonical value in seconds.
   * @return Time value with canonical storage set to @p raw.
   */
  static Time from_raw(double raw) noexcept;

private:
  explicit constexpr Time(double raw) noexcept : QuantityBase(raw) {}

  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Time> {
  size_t operator()(const cpp_helper_libs::quantities::Time &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_TIME_HPP
