#include "cpp_helper_libs/quantities/mass.hpp"

#include <array>
#include <utility>

#include "quantity_conversion_common.hpp"
namespace cpp_helper_libs::quantities {
namespace {

// Exact unit-to-kilogram conversion factors.
constexpr double kMilligramInKilograms = 1e-6;
constexpr double kGramInKilograms = 1e-3;
constexpr double kTonneInKilograms = 1000.0;
constexpr double kOunceInKilograms = 0.028349523125;
constexpr double kPoundInKilograms = 0.45359237;
constexpr std::array<std::pair<Mass::Unit, double>, 6> kToRawScales{{
    {Mass::Unit::Milligram, kMilligramInKilograms},
    {Mass::Unit::Gram, kGramInKilograms},
    {Mass::Unit::Kilogram, 1.0},
    {Mass::Unit::Tonne, kTonneInKilograms},
    {Mass::Unit::Ounce, kOunceInKilograms},
    {Mass::Unit::Pound, kPoundInKilograms},
}};

} // namespace

CPPHL_DEFINE_SCALED_QUANTITY_CORE_METHODS(Mass, kToRawScales)

Mass Mass::milligrams(const double value) { return Mass(value, Unit::Milligram); }
Mass Mass::grams(const double value) { return Mass(value, Unit::Gram); }
Mass Mass::kilograms(const double value) { return Mass(value, Unit::Kilogram); }
Mass Mass::tonnes(const double value) { return Mass(value, Unit::Tonne); }
Mass Mass::ounces(const double value) { return Mass(value, Unit::Ounce); }
Mass Mass::pounds(const double value) { return Mass(value, Unit::Pound); }

} // namespace cpp_helper_libs::quantities
