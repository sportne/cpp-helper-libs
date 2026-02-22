#include "cpp_helper_libs/quantities/angle.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Conversion constant between radians and degrees.
constexpr double kDegreesPerCircleFraction = 180.0;
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

} // namespace

Angle::Angle(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Angle Angle::radians(const double value) { return Angle(value, Unit::Radian); }
Angle Angle::degrees(const double value) { return Angle(value, Unit::Degree); }

Angle Angle::bound_zero_to_two_pi() const noexcept {
  double normalized = std::fmod(raw_value(), kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }

  return Angle::from_raw(normalized);
}

Angle Angle::bound_negative_pi_to_pi() const noexcept {
  double normalized = std::fmod(raw_value(), kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  if (normalized >= std::numbers::pi_v<double>) {
    normalized -= kTwoPi;
  }

  return Angle::from_raw(normalized);
}

double Angle::in(const Unit unit) const {
  switch (unit) {
  case Unit::Radian:
    return raw_value();
  case Unit::Degree:
    return raw_value() * (kDegreesPerCircleFraction / std::numbers::pi_v<double>);
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Angle::Unit value");
}

Angle Angle::from_raw(const double raw) noexcept { return Angle(raw); }

double Angle::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::Radian:
    return value;
  case Unit::Degree:
    return value * (std::numbers::pi_v<double> / kDegreesPerCircleFraction);
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Angle::Unit value");
}

} // namespace cpp_helper_libs::quantities
