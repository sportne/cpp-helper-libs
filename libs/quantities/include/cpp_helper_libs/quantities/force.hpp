// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_FORCE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_FORCE_HPP

#include <cstddef>
#include <string_view>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Physical force quantity stored canonically in newtons.
 */
class Force final : public QuantityBase<Force> {
public:
  /**
   * @brief Supported force units for construction and conversion.
   */
  enum class Unit {
    Newton,
    Kilonewton,
    PoundForce,
  };

  /**
   * @brief Construct a force value from a magnitude and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Force(double value, Unit unit);

  /**
   * @brief Create a force in newtons.
   *
   * @param value Magnitude in newtons.
   * @return Force value.
   */
  static Force newtons(double value);

  /**
   * @brief Create a force in kilonewtons.
   *
   * @param value Magnitude in kilonewtons.
   * @return Force value.
   */
  static Force kilonewtons(double value);

  /**
   * @brief Create a force in pound-force.
   *
   * @param value Magnitude in pound-force.
   * @return Force value.
   */
  static Force pound_force(double value);

  /**
   * @brief Convert this force quantity into another unit.
   *
   * @param unit Target force unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct a force quantity from canonical raw newtons.
   *
   * @param raw Canonical value in newtons.
   * @return Force value with canonical storage set to @p raw.
   */
  static Force from_raw(double raw) noexcept;

private:
  /// Construct directly from canonical raw newtons.
  explicit constexpr Force(double raw) noexcept : QuantityBase(raw) {}

  /// Convert from a user-facing unit into canonical raw newtons.
  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Force> {
  size_t operator()(const cpp_helper_libs::quantities::Force &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_FORCE_HPP
