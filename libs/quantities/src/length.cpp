#include "cpp_helper_libs/quantities/length.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
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
constexpr std::array<std::pair<Length::Unit, double>, 8> kToRawScales{{
    {Length::Unit::Millimeter, kMillimeterInMeters},
    {Length::Unit::Centimeter, kCentimeterInMeters},
    {Length::Unit::Meter, 1.0},
    {Length::Unit::Kilometer, kKilometerInMeters},
    {Length::Unit::Inch, kInchInMeters},
    {Length::Unit::Foot, kFootInMeters},
    {Length::Unit::Yard, kYardInMeters},
    {Length::Unit::Mile, kMileInMeters},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Length, kToRawScales)

Length Length::millimeters(const double value) { return Length(value, Unit::Millimeter); }
Length Length::centimeters(const double value) { return Length(value, Unit::Centimeter); }
Length Length::meters(const double value) { return Length(value, Unit::Meter); }
Length Length::kilometers(const double value) { return Length(value, Unit::Kilometer); }
Length Length::inches(const double value) { return Length(value, Unit::Inch); }
Length Length::feet(const double value) { return Length(value, Unit::Foot); }
Length Length::yards(const double value) { return Length(value, Unit::Yard); }
Length Length::miles(const double value) { return Length(value, Unit::Mile); }

} // namespace cpp_helper_libs::quantities
