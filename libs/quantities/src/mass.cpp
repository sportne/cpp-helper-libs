#include "cpp_helper_libs/quantities/mass.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-kilogram conversion factors.
constexpr double kMilligramInKilograms = 1e-6;
constexpr double kGramInKilograms = 1e-3;
constexpr double kTonneInKilograms = 1000.0;
constexpr double kOunceInKilograms = 0.028349523125;
constexpr double kPoundInKilograms = 0.45359237;

} // namespace

Mass::Mass(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Mass Mass::milligrams(const double value) { return Mass(value, Unit::Milligram); }
Mass Mass::grams(const double value) { return Mass(value, Unit::Gram); }
Mass Mass::kilograms(const double value) { return Mass(value, Unit::Kilogram); }
Mass Mass::tonnes(const double value) { return Mass(value, Unit::Tonne); }
Mass Mass::ounces(const double value) { return Mass(value, Unit::Ounce); }
Mass Mass::pounds(const double value) { return Mass(value, Unit::Pound); }

double Mass::in(const Unit unit) const {
  switch (unit) {
  case Unit::Milligram:
    return raw_value() / kMilligramInKilograms;
  case Unit::Gram:
    return raw_value() / kGramInKilograms;
  case Unit::Kilogram:
    return raw_value();
  case Unit::Tonne:
    return raw_value() / kTonneInKilograms;
  case Unit::Ounce:
    return raw_value() / kOunceInKilograms;
  case Unit::Pound:
    return raw_value() / kPoundInKilograms;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Mass::Unit value");
}

Mass Mass::from_raw(const double raw) noexcept { return Mass(raw); }

double Mass::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::Milligram:
    return value * kMilligramInKilograms;
  case Unit::Gram:
    return value * kGramInKilograms;
  case Unit::Kilogram:
    return value;
  case Unit::Tonne:
    return value * kTonneInKilograms;
  case Unit::Ounce:
    return value * kOunceInKilograms;
  case Unit::Pound:
    return value * kPoundInKilograms;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Mass::Unit value");
}

} // namespace cpp_helper_libs::quantities
