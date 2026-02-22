#ifndef CPP_HELPER_LIBS_QUANTITIES_FORCE_HPP
#define CPP_HELPER_LIBS_QUANTITIES_FORCE_HPP

#include <cstddef>
#include <functional>

#include "cpp_helper_libs/quantities/quantity_base.hpp"

namespace cpp_helper_libs::quantities {

class Force final : public QuantityBase<Force> {
public:
  enum class Unit {
    Newton,
    Kilonewton,
    PoundForce,
  };

  explicit Force(double value, Unit unit);

  static Force newtons(double value);
  static Force kilonewtons(double value);
  static Force pound_force(double value);

  double in(Unit unit) const;

  static Force from_raw(double raw) noexcept;

private:
  explicit constexpr Force(double raw) noexcept : QuantityBase(raw) {}

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
