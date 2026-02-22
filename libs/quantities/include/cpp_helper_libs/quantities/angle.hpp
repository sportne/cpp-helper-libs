#ifndef CPP_HELPER_LIBS_QUANTITIES_ANGLE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_ANGLE_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Angle final : public QuantityBase<Angle> {
public:
  enum class Unit {
    Radian,
    Degree,
  };

  explicit Angle(double value, Unit unit);

  static Angle radians(double value);
  static Angle degrees(double value);

  // Return an equivalent angle normalized to the range [0, 2*pi).
  Angle bound_zero_to_two_pi() const noexcept;

  // Return an equivalent angle normalized to the range [-pi, pi).
  Angle bound_negative_pi_to_pi() const noexcept;

  double in(Unit unit) const;

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
