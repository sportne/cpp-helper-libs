#include "cpp_helper_libs/quantities/speed.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-meters/second conversion factors.
constexpr double kKilometerInMeters = 1000.0;
constexpr double kHourInSeconds = 3600.0;
constexpr double kKilometersPerHourInMetersPerSecond = kKilometerInMeters / kHourInSeconds;
constexpr double kMilePerHourInMetersPerSecond = 0.44704;
constexpr double kFootPerSecondInMetersPerSecond = 0.3048;

} // namespace

Speed::Speed(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Speed Speed::meters_per_second(const double value) { return Speed(value, Unit::MetersPerSecond); }
Speed Speed::kilometers_per_hour(const double value) {
  return Speed(value, Unit::KilometersPerHour);
}
Speed Speed::miles_per_hour(const double value) { return Speed(value, Unit::MilesPerHour); }
Speed Speed::feet_per_second(const double value) { return Speed(value, Unit::FeetPerSecond); }

double Speed::in(const Unit unit) const {
  switch (unit) {
  case Unit::MetersPerSecond:
    return raw_value();
  case Unit::KilometersPerHour:
    return raw_value() / kKilometersPerHourInMetersPerSecond;
  case Unit::MilesPerHour:
    return raw_value() / kMilePerHourInMetersPerSecond;
  case Unit::FeetPerSecond:
    return raw_value() / kFootPerSecondInMetersPerSecond;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Speed::Unit value");
}

Speed Speed::from_raw(const double raw) noexcept { return Speed(raw); }

double Speed::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::MetersPerSecond:
    return value;
  case Unit::KilometersPerHour:
    return value * kKilometersPerHourInMetersPerSecond;
  case Unit::MilesPerHour:
    return value * kMilePerHourInMetersPerSecond;
  case Unit::FeetPerSecond:
    return value * kFootPerSecondInMetersPerSecond;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Speed::Unit value");
}

} // namespace cpp_helper_libs::quantities
