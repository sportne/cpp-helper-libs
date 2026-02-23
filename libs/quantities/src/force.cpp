#include "cpp_helper_libs/quantities/force.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-newton conversion factors.
constexpr double kKilonewtonInNewtons = 1000.0;
constexpr double kPoundForceInNewtons = 4.4482216152605;
constexpr std::array<std::pair<Force::Unit, double>, 3> kToRawScales{{
    {Force::Unit::Newton, 1.0},
    {Force::Unit::Kilonewton, kKilonewtonInNewtons},
    {Force::Unit::PoundForce, kPoundForceInNewtons},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Force, kToRawScales)

Force Force::newtons(const double value) { return Force(value, Unit::Newton); }
Force Force::kilonewtons(const double value) { return Force(value, Unit::Kilonewton); }
Force Force::pound_force(const double value) { return Force(value, Unit::PoundForce); }

} // namespace cpp_helper_libs::quantities
