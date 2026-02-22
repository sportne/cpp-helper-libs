#ifndef CPP_HELPER_LIBS_QUANTITIES_TIME_HPP
#define CPP_HELPER_LIBS_QUANTITIES_TIME_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Time final : public QuantityBase<Time> {
public:
  enum class Unit {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Hour,
  };

  explicit Time(double value, Unit unit);

  static Time nanoseconds(double value);
  static Time microseconds(double value);
  static Time milliseconds(double value);
  static Time seconds(double value);
  static Time minutes(double value);
  static Time hours(double value);

  double in(Unit unit) const;

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
