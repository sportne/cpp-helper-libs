#include "cpp_helper_libs/quantities/speed.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-meters/second conversion factors.
constexpr double kKilometerInMeters = 1000.0;
constexpr double kHourInSeconds = 3600.0;
constexpr double kKilometersPerHourInMetersPerSecond = kKilometerInMeters / kHourInSeconds;
constexpr double kMilePerHourInMetersPerSecond = 0.44704;
constexpr double kFootPerSecondInMetersPerSecond = 0.3048;
constexpr std::array<std::pair<Speed::Unit, double>, 4> kToRawScales{{
    {Speed::Unit::MetersPerSecond, 1.0},
    {Speed::Unit::KilometersPerHour, kKilometersPerHourInMetersPerSecond},
    {Speed::Unit::MilesPerHour, kMilePerHourInMetersPerSecond},
    {Speed::Unit::FeetPerSecond, kFootPerSecondInMetersPerSecond},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Speed, kToRawScales)

Speed Speed::meters_per_second(const double value) { return Speed(value, Unit::MetersPerSecond); }
Speed Speed::kilometers_per_hour(const double value) {
  return Speed(value, Unit::KilometersPerHour);
}
Speed Speed::miles_per_hour(const double value) { return Speed(value, Unit::MilesPerHour); }
Speed Speed::feet_per_second(const double value) { return Speed(value, Unit::FeetPerSecond); }

} // namespace cpp_helper_libs::quantities
