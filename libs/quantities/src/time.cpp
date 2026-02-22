#include "cpp_helper_libs/quantities/time.hpp"

#include <stdexcept>

namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-second conversion factors.
constexpr double kNanosecondInSeconds = 1e-9;
constexpr double kMicrosecondInSeconds = 1e-6;
constexpr double kMillisecondInSeconds = 1e-3;
constexpr double kMinuteInSeconds = 60.0;
constexpr double kHourInSeconds = 3600.0;

} // namespace

Time::Time(const double value, const Unit unit) : QuantityBase(to_raw(value, unit)) {}

Time Time::nanoseconds(const double value) { return Time(value, Unit::Nanosecond); }
Time Time::microseconds(const double value) { return Time(value, Unit::Microsecond); }
Time Time::milliseconds(const double value) { return Time(value, Unit::Millisecond); }
Time Time::seconds(const double value) { return Time(value, Unit::Second); }
Time Time::minutes(const double value) { return Time(value, Unit::Minute); }
Time Time::hours(const double value) { return Time(value, Unit::Hour); }

double Time::in(const Unit unit) const {
  switch (unit) {
  case Unit::Nanosecond:
    return raw_value() / kNanosecondInSeconds;
  case Unit::Microsecond:
    return raw_value() / kMicrosecondInSeconds;
  case Unit::Millisecond:
    return raw_value() / kMillisecondInSeconds;
  case Unit::Second:
    return raw_value();
  case Unit::Minute:
    return raw_value() / kMinuteInSeconds;
  case Unit::Hour:
    return raw_value() / kHourInSeconds;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Time::Unit value");
}

Time Time::from_raw(const double raw) noexcept { return Time(raw); }

double Time::to_raw(const double value, const Unit unit) {
  switch (unit) {
  case Unit::Nanosecond:
    return value * kNanosecondInSeconds;
  case Unit::Microsecond:
    return value * kMicrosecondInSeconds;
  case Unit::Millisecond:
    return value * kMillisecondInSeconds;
  case Unit::Second:
    return value;
  case Unit::Minute:
    return value * kMinuteInSeconds;
  case Unit::Hour:
    return value * kHourInSeconds;
  }

  // Catch corrupted/invalid enum values from external callers.
  throw std::invalid_argument("Invalid Time::Unit value");
}

} // namespace cpp_helper_libs::quantities
