#ifndef CPP_HELPER_LIBS_QUANTITIES_QUANTITY_BASE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_QUANTITY_BASE_HPP

#include <cstddef>
#include <functional>
#include <stdexcept>

namespace cpp_helper_libs::quantities {

template <typename Derived> class QuantityBase {
public:
  // Quantities compare on canonical raw values (for example meters, seconds).
  bool operator==(const QuantityBase &other) const noexcept { return raw_ == other.raw_; }

  bool operator!=(const QuantityBase &other) const noexcept { return !(*this == other); }

  bool operator<(const QuantityBase &other) const noexcept { return raw_ < other.raw_; }

  bool operator<=(const QuantityBase &other) const noexcept { return raw_ <= other.raw_; }

  bool operator>(const QuantityBase &other) const noexcept { return raw_ > other.raw_; }

  bool operator>=(const QuantityBase &other) const noexcept { return raw_ >= other.raw_; }

  // Arithmetic returns the derived quantity while preserving canonical storage.
  Derived operator+(const QuantityBase &other) const noexcept {
    return Derived::from_raw(raw_ + other.raw_);
  }

  Derived operator-(const QuantityBase &other) const noexcept {
    return Derived::from_raw(raw_ - other.raw_);
  }

  // Scale by a scalar and return a new quantity.
  Derived multiplied_by(const double factor) const noexcept {
    return Derived::from_raw(raw_ * factor);
  }

  // Divide by a scalar and return a new quantity.
  Derived divided_by(const double divisor) const {
    if (divisor == 0.0) {
      throw std::invalid_argument("Cannot divide quantity by zero");
    }

    return Derived::from_raw(raw_ / divisor);
  }

  // Normalize signed zero so +0.0 and -0.0 hash the same.
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
