#ifndef CPP_HELPER_LIBS_QUANTITIES_LENGTH_HPP
#define CPP_HELPER_LIBS_QUANTITIES_LENGTH_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Length final : public QuantityBase<Length> {
public:
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

  explicit Length(double value, Unit unit);

  static Length millimeters(double value);
  static Length centimeters(double value);
  static Length meters(double value);
  static Length kilometers(double value);
  static Length inches(double value);
  static Length feet(double value);
  static Length yards(double value);
  static Length miles(double value);

  double in(Unit unit) const;

  static Length from_raw(double raw) noexcept;

private:
  explicit constexpr Length(double raw) noexcept : QuantityBase(raw) {}

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
