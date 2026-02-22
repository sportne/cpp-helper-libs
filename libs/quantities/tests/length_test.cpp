#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "cpp_helper_libs/quantities/length.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Length;
using cpp_helper_libs::quantities::test::make_invalid_enum;

TEST(LengthTest, ConvertsAndComparesAcrossUnits) {
  const Length one_mile = Length::miles(1.0);
  const Length meters = Length::meters(1609.344);

  EXPECT_EQ(one_mile, meters);
  EXPECT_DOUBLE_EQ(one_mile.in(Length::Unit::Foot), 5280.0);
  EXPECT_GT(Length::kilometers(2.0), Length::meters(1500.0));
}

TEST(LengthTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneMeter = 1.0;
  constexpr double kMillimetersPerMeter = 1000.0;
  constexpr double kCentimetersPerMeter = 100.0;
  constexpr double kKilometersPerMeter = 0.001;
  constexpr double kInchesPerMeter = 39.37007874015748;
  constexpr double kFeetPerMeter = 3.280839895013123;
  constexpr double kYardsPerMeter = 1.0936132983377078;
  constexpr double kMilesPerMeter = 0.0006213711922373339;

  const Length one_meter = Length::meters(kOneMeter);

  EXPECT_NEAR(one_meter.in(Length::Unit::Millimeter), kMillimetersPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Centimeter), kCentimetersPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Meter), kOneMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Kilometer), kKilometersPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Inch), kInchesPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Foot), kFeetPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Yard), kYardsPerMeter, kTolerance);
  EXPECT_NEAR(one_meter.in(Length::Unit::Mile), kMilesPerMeter, kTolerance);

  EXPECT_NEAR(Length::millimeters(kMillimetersPerMeter).in(Length::Unit::Meter), kOneMeter,
              kTolerance);
  EXPECT_NEAR(Length::centimeters(kCentimetersPerMeter).in(Length::Unit::Meter), kOneMeter,
              kTolerance);
  EXPECT_NEAR(Length::meters(kOneMeter).in(Length::Unit::Meter), kOneMeter, kTolerance);
  EXPECT_NEAR(Length::kilometers(kKilometersPerMeter).in(Length::Unit::Meter), kOneMeter,
              kTolerance);
  EXPECT_NEAR(Length::inches(kInchesPerMeter).in(Length::Unit::Meter), kOneMeter, kTolerance);
  EXPECT_NEAR(Length::feet(kFeetPerMeter).in(Length::Unit::Meter), kOneMeter, kTolerance);
  EXPECT_NEAR(Length::yards(kYardsPerMeter).in(Length::Unit::Meter), kOneMeter, kTolerance);
  EXPECT_NEAR(Length::miles(kMilesPerMeter).in(Length::Unit::Meter), kOneMeter, kTolerance);
}

TEST(LengthTest, EqualQuantitiesProduceEqualHashes) {
  const Length meters = Length::meters(1000.0);
  const Length kilometers = Length::kilometers(1.0);

  EXPECT_EQ(meters.hash_code(), kilometers.hash_code());

  std::unordered_set<Length> values;
  values.insert(meters);
  values.insert(kilometers);

  EXPECT_EQ(values.size(), 1U);
}

TEST(LengthTest, SupportsMultiplyAndDivide) {
  constexpr double kScaleUp = 3.0;
  constexpr double kScaleDown = 2.0;

  const Length base = Length::meters(2.0);
  const Length scaled_up = base.multiplied_by(kScaleUp);
  const Length scaled_down = scaled_up.divided_by(kScaleDown);

  EXPECT_EQ(scaled_up, Length::meters(6.0));
  EXPECT_EQ(scaled_down, Length::meters(3.0));
}

TEST(LengthTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Length::Unit>();
  EXPECT_THROW(static_cast<void>(Length(1.0, invalid_unit)), std::invalid_argument);
}

TEST(LengthTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Length::Unit>();
  const Length value = Length::meters(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
