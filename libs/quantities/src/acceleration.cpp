#include "cpp_helper_libs/quantities/acceleration.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-meters/second^2 conversion factors.
constexpr double kFootPerSecondSquaredInMetersPerSecondSquared = 0.3048;
constexpr double kStandardGravityInMetersPerSecondSquared = 9.80665;
constexpr std::array<std::pair<Acceleration::Unit, double>, 3> kToRawScales{{
    {Acceleration::Unit::MetersPerSecondSquared, 1.0},
    {Acceleration::Unit::FeetPerSecondSquared, kFootPerSecondSquaredInMetersPerSecondSquared},
    {Acceleration::Unit::StandardGravity, kStandardGravityInMetersPerSecondSquared},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Acceleration, kToRawScales)

Acceleration Acceleration::meters_per_second_squared(const double value) {
  return Acceleration(value, Unit::MetersPerSecondSquared);
}

Acceleration Acceleration::feet_per_second_squared(const double value) {
  return Acceleration(value, Unit::FeetPerSecondSquared);
}

Acceleration Acceleration::standard_gravity(const double value) {
  return Acceleration(value, Unit::StandardGravity);
}

} // namespace cpp_helper_libs::quantities
