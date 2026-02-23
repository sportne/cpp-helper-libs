#include "cpp_helper_libs/quantities/time.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-second conversion factors.
constexpr double kNanosecondInSeconds = 1e-9;
constexpr double kMicrosecondInSeconds = 1e-6;
constexpr double kMillisecondInSeconds = 1e-3;
constexpr double kMinuteInSeconds = 60.0;
constexpr double kHourInSeconds = 3600.0;
constexpr std::array<std::pair<Time::Unit, double>, 6> kToRawScales{{
    {Time::Unit::Nanosecond, kNanosecondInSeconds},
    {Time::Unit::Microsecond, kMicrosecondInSeconds},
    {Time::Unit::Millisecond, kMillisecondInSeconds},
    {Time::Unit::Second, 1.0},
    {Time::Unit::Minute, kMinuteInSeconds},
    {Time::Unit::Hour, kHourInSeconds},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Time, kToRawScales)

Time Time::nanoseconds(const double value) { return Time(value, Unit::Nanosecond); }
Time Time::microseconds(const double value) { return Time(value, Unit::Microsecond); }
Time Time::milliseconds(const double value) { return Time(value, Unit::Millisecond); }
Time Time::seconds(const double value) { return Time(value, Unit::Second); }
Time Time::minutes(const double value) { return Time(value, Unit::Minute); }
Time Time::hours(const double value) { return Time(value, Unit::Hour); }

} // namespace cpp_helper_libs::quantities
