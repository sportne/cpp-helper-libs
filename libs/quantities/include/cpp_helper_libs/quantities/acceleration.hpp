#ifndef CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP
#define CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Acceleration final : public QuantityBase<Acceleration> {
public:
  enum class Unit {
    MetersPerSecondSquared,
    FeetPerSecondSquared,
    StandardGravity,
  };

  explicit Acceleration(double value, Unit unit);

  static Acceleration meters_per_second_squared(double value);
  static Acceleration feet_per_second_squared(double value);
  static Acceleration standard_gravity(double value);

  double in(Unit unit) const;

  static Acceleration from_raw(double raw) noexcept;

private:
  explicit constexpr Acceleration(double raw) noexcept : QuantityBase(raw) {}

  static double to_raw(double value, Unit unit);
};

} // namespace cpp_helper_libs::quantities

namespace std {

template <> struct hash<cpp_helper_libs::quantities::Acceleration> {
  size_t operator()(const cpp_helper_libs::quantities::Acceleration &value) const noexcept {
    return value.hash_code();
  }
};

} // namespace std

#endif // CPP_HELPER_LIBS_QUANTITIES_ACCELERATION_HPP
