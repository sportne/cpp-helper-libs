#include "cpp_helper_libs/quantities/acceleration.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-meters/second^2 conversion factors.
constexpr double kFootPerSecondSquaredInMetersPerSecondSquared = 0.3048;
constexpr double kStandardGravityInMetersPerSecondSquared = 9.80665;

} // namespace

Acceleration::Acceleration(const double value, const Unit unit)
    : QuantityBase(to_raw(value, unit)) {}

Acceleration Acceleration::meters_per_second_squared(const double value) {
  return Acceleration(value, Unit::MetersPerSecondSquared);
}

Acceleration Acceleration::feet_per_second_squared(const double value) {
  return Acceleration(value, Unit::FeetPerSecondSquared);
}

Acceleration Acceleration::standard_gravity(const double value) {
  return Acceleration(value, Unit::StandardGravity);
}

double Acceleration::in(const Unit unit) const {
  switch (unit) {
  case Unit::MetersPerSecondSquared:
    return raw_value();
  case Unit::FeetPerSecondSquared:
    return raw_value() / kFootPerSecondSquaredInMetersPerSecondSquared;
  case Unit::StandardGravity:
    return raw_value() / kStandardGravityInMetersPerSecondSquared;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Acceleration::Unit value");
}

Acceleration Acceleration::from_raw(const double raw) noexcept { return Acceleration(raw); }

double Acceleration::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::MetersPerSecondSquared:
    return value;
  case Unit::FeetPerSecondSquared:
    return value * kFootPerSecondSquaredInMetersPerSecondSquared;
  case Unit::StandardGravity:
    return value * kStandardGravityInMetersPerSecondSquared;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Acceleration::Unit value");
}

} // namespace cpp_helper_libs::quantities
