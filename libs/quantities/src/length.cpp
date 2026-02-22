#include "cpp_helper_libs/quantities/length.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-meter conversion factors.
constexpr double kMillimeterInMeters = 0.001;
constexpr double kCentimeterInMeters = 0.01;
constexpr double kKilometerInMeters = 1000.0;
constexpr double kInchInMeters = 0.0254;
constexpr double kFootInMeters = 0.3048;
constexpr double kYardInMeters = 0.9144;
constexpr double kMileInMeters = 1609.344;

} // namespace

Length::Length(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Length Length::millimeters(const double value) { return Length(value, Unit::Millimeter); }
Length Length::centimeters(const double value) { return Length(value, Unit::Centimeter); }
Length Length::meters(const double value) { return Length(value, Unit::Meter); }
Length Length::kilometers(const double value) { return Length(value, Unit::Kilometer); }
Length Length::inches(const double value) { return Length(value, Unit::Inch); }
Length Length::feet(const double value) { return Length(value, Unit::Foot); }
Length Length::yards(const double value) { return Length(value, Unit::Yard); }
Length Length::miles(const double value) { return Length(value, Unit::Mile); }

double Length::in(const Unit unit) const {
  switch (unit) {
  case Unit::Millimeter:
    return raw_value() / kMillimeterInMeters;
  case Unit::Centimeter:
    return raw_value() / kCentimeterInMeters;
  case Unit::Meter:
    return raw_value();
  case Unit::Kilometer:
    return raw_value() / kKilometerInMeters;
  case Unit::Inch:
    return raw_value() / kInchInMeters;
  case Unit::Foot:
    return raw_value() / kFootInMeters;
  case Unit::Yard:
    return raw_value() / kYardInMeters;
  case Unit::Mile:
    return raw_value() / kMileInMeters;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Length::Unit value");
}

Length Length::from_raw(const double raw) noexcept { return Length(raw); }

double Length::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::Millimeter:
    return value * kMillimeterInMeters;
  case Unit::Centimeter:
    return value * kCentimeterInMeters;
  case Unit::Meter:
    return value;
  case Unit::Kilometer:
    return value * kKilometerInMeters;
  case Unit::Inch:
    return value * kInchInMeters;
  case Unit::Foot:
    return value * kFootInMeters;
  case Unit::Yard:
    return value * kYardInMeters;
  case Unit::Mile:
    return value * kMileInMeters;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Length::Unit value");
}

} // namespace cpp_helper_libs::quantities
