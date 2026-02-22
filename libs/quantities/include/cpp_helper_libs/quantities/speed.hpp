#ifndef CPP_HELPER_LIBS_QUANTITIES_SPEED_HPP
#define CPP_HELPER_LIBS_QUANTITIES_SPEED_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Speed final : public QuantityBase<Speed> {
public:
  enum class Unit {
    MetersPerSecond,
    KilometersPerHour,
    MilesPerHour,
    FeetPerSecond,
  };

  explicit Speed(double value, Unit unit);

  static Speed meters_per_second(double value);
  static Speed kilometers_per_hour(double value);
  static Speed miles_per_hour(double value);
  static Speed feet_per_second(double value);

  double in(Unit unit) const;

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
