#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "cpp_helper_libs/quantities/mass.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Mass;
using cpp_helper_libs::quantities::test::make_invalid_enum;

TEST(MassTest, ConvertsAndOrdersValues) {
  const Mass pounds = Mass::pounds(2.0);
  const Mass kilograms = Mass::kilograms(0.90718474);

  EXPECT_EQ(pounds, kilograms);
  EXPECT_LT(Mass::grams(500.0), Mass::kilograms(1.0));
}

TEST(MassTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneKilogram = 1.0;
  constexpr double kMilligramsPerKilogram = 1000000.0;
  constexpr double kGramsPerKilogram = 1000.0;
  constexpr double kTonnesPerKilogram = 0.001;
  constexpr double kOuncesPerKilogram = 35.27396194958041;
  constexpr double kPoundsPerKilogram = 2.204622621848776;

  const Mass one_kilogram = Mass::kilograms(kOneKilogram);

  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Milligram), kMilligramsPerKilogram, kTolerance);
  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Gram), kGramsPerKilogram, kTolerance);
  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Tonne), kTonnesPerKilogram, kTolerance);
  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Ounce), kOuncesPerKilogram, kTolerance);
  EXPECT_NEAR(one_kilogram.in(Mass::Unit::Pound), kPoundsPerKilogram, kTolerance);

  EXPECT_NEAR(Mass::milligrams(kMilligramsPerKilogram).in(Mass::Unit::Kilogram), kOneKilogram,
              kTolerance);
  EXPECT_NEAR(Mass::grams(kGramsPerKilogram).in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
  EXPECT_NEAR(Mass::kilograms(kOneKilogram).in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
  EXPECT_NEAR(Mass::tonnes(kTonnesPerKilogram).in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
  EXPECT_NEAR(Mass::ounces(kOuncesPerKilogram).in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
  EXPECT_NEAR(Mass::pounds(kPoundsPerKilogram).in(Mass::Unit::Kilogram), kOneKilogram, kTolerance);
}

TEST(MassTest, SupportsRawFactory) {
  const Mass value = Mass::from_raw(42.0);
  EXPECT_DOUBLE_EQ(value.in(Mass::Unit::Kilogram), 42.0);
}

TEST(MassTest, HashSpecializationIsUsable) {
  constexpr std::size_t kExpectedSingleEntry = 1U;

  std::unordered_set<Mass> values;
  const Mass sample = Mass::kilograms(1.0);
  values.insert(sample);
  values.insert(sample);
  EXPECT_EQ(values.size(), kExpectedSingleEntry);
}

TEST(MassTest, ThrowsOnDivideByZero) {
  constexpr double kZero = 0.0;

  const Mass mass = Mass::kilograms(5.0);
  EXPECT_THROW(static_cast<void>(mass.divided_by(kZero)), std::invalid_argument);
}

TEST(MassTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Mass::Unit>();
  EXPECT_THROW(static_cast<void>(Mass(1.0, invalid_unit)), std::invalid_argument);
}

TEST(MassTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Mass::Unit>();
  const Mass value = Mass::kilograms(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
