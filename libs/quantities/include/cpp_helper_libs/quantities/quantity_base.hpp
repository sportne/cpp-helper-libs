// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_QUANTITIES_QUANTITY_BASE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_QUANTITY_BASE_HPP

#include <cstddef>
#include <functional>
#include <stdexcept>

namespace cpp_helper_libs::quantities {

/**
 * @brief CRTP base class for strongly-typed scalar quantities.
 *
 * @tparam Derived Concrete quantity type implementing:
 *   - `static Derived from_raw(double)`
 *
 * Quantities are stored in a canonical raw unit (for example, meters or seconds).
 */
template <typename Derived> class QuantityBase {
public:
  /**
   * @brief Compare two quantities for exact raw-value equality.
   *
   * @param other Quantity to compare against.
   * @return `true` when both quantities have identical canonical raw values.
   */
  bool operator==(const QuantityBase &other) const noexcept { return raw_ == other.raw_; }

  /**
   * @brief Compare two quantities for raw-value inequality.
   *
   * @param other Quantity to compare against.
   * @return `true` when canonical raw values differ.
   */
  bool operator!=(const QuantityBase &other) const noexcept { return !(*this == other); }

  /**
   * @brief Compare whether this quantity is strictly less than another.
   *
   * @param other Quantity to compare against.
   * @return `true` when this canonical raw value is less than @p other.
   */
  bool operator<(const QuantityBase &other) const noexcept { return raw_ < other.raw_; }

  /**
   * @brief Compare whether this quantity is less than or equal to another.
   *
   * @param other Quantity to compare against.
   * @return `true` when this canonical raw value is <= @p other.
   */
  bool operator<=(const QuantityBase &other) const noexcept { return raw_ <= other.raw_; }

  /**
   * @brief Compare whether this quantity is strictly greater than another.
   *
   * @param other Quantity to compare against.
   * @return `true` when this canonical raw value is greater than @p other.
   */
  bool operator>(const QuantityBase &other) const noexcept { return raw_ > other.raw_; }

  /**
   * @brief Compare whether this quantity is greater than or equal to another.
   *
   * @param other Quantity to compare against.
   * @return `true` when this canonical raw value is >= @p other.
   */
  bool operator>=(const QuantityBase &other) const noexcept { return raw_ >= other.raw_; }

  /**
   * @brief Add two quantities of the same derived type.
   *
   * @param other Quantity to add.
   * @return New quantity with canonical raw value `this->raw + other.raw`.
   */
  Derived operator+(const QuantityBase &other) const noexcept {
    return Derived::from_raw(raw_ + other.raw_);
  }

  /**
   * @brief Subtract one quantity from another of the same derived type.
   *
   * @param other Quantity to subtract.
   * @return New quantity with canonical raw value `this->raw - other.raw`.
   */
  Derived operator-(const QuantityBase &other) const noexcept {
    return Derived::from_raw(raw_ - other.raw_);
  }

  /**
   * @brief Scale a quantity by a scalar factor.
   *
   * @param factor Scalar multiplier.
   * @return New quantity with canonical raw value `raw * factor`.
   */
  Derived multiplied_by(const double factor) const noexcept {
    return Derived::from_raw(raw_ * factor);
  }

  /**
   * @brief Divide a quantity by a scalar divisor.
   *
   * @param divisor Scalar divisor.
   * @return New quantity with canonical raw value `raw / divisor`.
   * @pre `divisor != 0.0`.
   * @throws std::invalid_argument When @p divisor is zero.
   */
  Derived divided_by(const double divisor) const {
    if (divisor == 0.0) {
      throw std::invalid_argument("Cannot divide quantity by zero");
    }

    return Derived::from_raw(raw_ / divisor);
  }

  /**
   * @brief Compute a stable hash code for the canonical raw value.
   *
   * Signed zeros are normalized so `+0.0` and `-0.0` hash identically.
   *
   * @return Hash value usable with `std::hash`-style containers.
   */
  std::size_t hash_code() const noexcept {
    const double normalized = (raw_ == 0.0) ? 0.0 : raw_;
    return std::hash<double>{}(normalized);
  }

protected:
  explicit constexpr QuantityBase(const double raw) noexcept : raw_(raw) {}

  double raw_value() const noexcept { return raw_; }

private:
  const double raw_;
};

} // namespace cpp_helper_libs::quantities

#endif // CPP_HELPER_LIBS_QUANTITIES_QUANTITY_BASE_HPP
