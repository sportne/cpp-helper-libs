#ifndef CPP_HELPER_LIBS_QUANTITIES_MASS_HPP
#define CPP_HELPER_LIBS_QUANTITIES_MASS_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Mass final : public QuantityBase<Mass> {
public:
  enum class Unit {
    Milligram,
    Gram,
    Kilogram,
    Tonne,
    Ounce,
    Pound,
  };

  explicit Mass(double value, Unit unit);

  static Mass milligrams(double value);
  static Mass grams(double value);
  static Mass kilograms(double value);
  static Mass tonnes(double value);
  static Mass ounces(double value);
  static Mass pounds(double value);

  double in(Unit unit) const;

  static Mass from_raw(double raw) noexcept;

private:
  explicit constexpr Mass(double raw) noexcept : QuantityBase(raw) {}

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
