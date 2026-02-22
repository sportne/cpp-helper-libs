// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_ANGLE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_ANGLE_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

/**
 * @brief Angular quantity stored canonically in radians.
 */
class Angle final : public QuantityBase<Angle> {
public:
  /**
   * @brief Supported angle units for construction and conversion.
   */
  enum class Unit {
    Radian,
    Degree,
  };

  /**
   * @brief Construct an angle from a value and unit.
   *
   * @param value Numeric magnitude in @p unit.
   * @param unit Unit associated with @p value.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  explicit Angle(double value, Unit unit);

  /**
   * @brief Create an angle in radians.
   *
   * @param value Magnitude in radians.
   * @return Angle value.
   */
  static Angle radians(double value);

  /**
   * @brief Create an angle in degrees.
   *
   * @param value Magnitude in degrees.
   * @return Angle value.
   */
  static Angle degrees(double value);

  /**
   * @brief Normalize to the range [0, 2*pi).
   *
   * @return Equivalent wrapped angle in [0, 2*pi).
   */
  Angle bound_zero_to_two_pi() const noexcept;

  /**
   * @brief Normalize to the range [-pi, pi).
   *
   * @return Equivalent wrapped angle in [-pi, pi).
   */
  Angle bound_negative_pi_to_pi() const noexcept;

  /**
   * @brief Convert this angle into another unit.
   *
   * @param unit Target angle unit.
   * @return Magnitude expressed in @p unit.
   * @throws std::invalid_argument When @p unit is not a valid enum value.
   */
  double in(Unit unit) const;

  /**
   * @brief Construct an angle directly from canonical raw radians.
   *
   * @param raw Canonical value in radians.
   * @return Angle value with canonical storage set to @p raw.
   */
  static Angle from_raw(double raw) noexcept;

private:
  explicit constexpr Angle(double raw) noexcept : QuantityBase(raw) {}

  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Angle> {
  size_t operator()(const cpp_helper_libs::quantities::Angle &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_ANGLE_HPP
