// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_MASS_HPP
#define CPP_HELPER_LIBS_QUANTITIES_MASS_HPP

#include <cstddef>
#include <string_view>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical mass quantity stored canonically in kilograms.
 */
class Mass final : public QuantityBase<Mass> {
public:
  /**
   * @brief Supported mass units for construction and conversion.
   */
  enum class Unit {
    Milligram,
    Gram,
    Kilogram,
    Tonne,
    Ounce,
    Pound,
  };

  /**
   * @brief Construct a mass value from a magnitude and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Mass(double value, Unit unit);

  /**
   * @brief Create a mass in milligrams.
   *
   * @param value Magnitude in milligrams.
   * @return Mass value.
   */
  static Mass milligrams(double value);

  /**
   * @brief Create a mass in grams.
   *
   * @param value Magnitude in grams.
   * @return Mass value.
   */
  static Mass grams(double value);

  /**
   * @brief Create a mass in kilograms.
   *
   * @param value Magnitude in kilograms.
   * @return Mass value.
   */
  static Mass kilograms(double value);

  /**
   * @brief Create a mass in tonnes.
   *
   * @param value Magnitude in tonnes.
   * @return Mass value.
   */
  static Mass tonnes(double value);

  /**
   * @brief Create a mass in ounces.
   *
   * @param value Magnitude in ounces.
   * @return Mass value.
   */
  static Mass ounces(double value);

  /**
   * @brief Create a mass in pounds.
   *
   * @param value Magnitude in pounds.
   * @return Mass value.
   */
  static Mass pounds(double value);

  /**
   * @brief Convert this mass quantity into another unit.
   *
   * @param unit Target mass unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct a mass quantity from canonical raw kilograms.
   *
   * @param raw Canonical value in kilograms.
   * @return Mass value with canonical storage set to @p raw.
   */
  static Mass from_raw(double raw) noexcept;

private:
  /// Construct directly from canonical raw kilograms.
  explicit constexpr Mass(double raw) noexcept : QuantityBase(raw) {}

  /// Convert from a user-facing unit into canonical raw kilograms.
  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Mass> {
  size_t operator()(const cpp_helper_libs::quantities::Mass &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_MASS_HPP
