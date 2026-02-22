#include "cpp_helper_libs/quantities/force.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-newton conversion factors.
constexpr double kKilonewtonInNewtons = 1000.0;
constexpr double kPoundForceInNewtons = 4.4482216152605;

} // namespace

Force::Force(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Force Force::newtons(const double value) { return Force(value, Unit::Newton); }
Force Force::kilonewtons(const double value) { return Force(value, Unit::Kilonewton); }
Force Force::pound_force(const double value) { return Force(value, Unit::PoundForce); }

double Force::in(const Unit unit) const {
  switch (unit) {
  case Unit::Newton:
    return raw_value();
  case Unit::Kilonewton:
    return raw_value() / kKilonewtonInNewtons;
  case Unit::PoundForce:
    return raw_value() / kPoundForceInNewtons;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Force::Unit value");
}

Force Force::from_raw(const double raw) noexcept { return Force(raw); }

double Force::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::Newton:
    return value;
  case Unit::Kilonewton:
    return value * kKilonewtonInNewtons;
  case Unit::PoundForce:
    return value * kPoundForceInNewtons;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Force::Unit value");
}

} // namespace cpp_helper_libs::quantities
