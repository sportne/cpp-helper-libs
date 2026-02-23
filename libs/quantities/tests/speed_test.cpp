#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "cpp_helper_libs/quantities/speed.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Speed;
using cpp_helper_libs::quantities::test::make_invalid_enum;

TEST(SpeedTest, HandlesDifferentUnits) {
  const Speed mph = Speed::miles_per_hour(60.0);

  EXPECT_DOUBLE_EQ(mph.in(Speed::Unit::MetersPerSecond), 26.8224);
  EXPECT_EQ(Speed::kilometers_per_hour(36.0), Speed::meters_per_second(10.0));
}

TEST(SpeedTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneMeterPerSecond = 1.0;
  constexpr double kKilometersPerHourPerMeterPerSecond = 3.6;
  constexpr double kMilesPerHourPerMeterPerSecond = 2.236936292054402;
  constexpr double kFeetPerSecondPerMeterPerSecond = 3.280839895013123;

  const Speed one_meter_per_second = Speed::meters_per_second(kOneMeterPerSecond);

  EXPECT_NEAR(one_meter_per_second.in(Speed::Unit::MetersPerSecond), kOneMeterPerSecond,
              kTolerance);
  EXPECT_NEAR(one_meter_per_second.in(Speed::Unit::KilometersPerHour),
              kKilometersPerHourPerMeterPerSecond, kTolerance);
  EXPECT_NEAR(one_meter_per_second.in(Speed::Unit::MilesPerHour), kMilesPerHourPerMeterPerSecond,
              kTolerance);
  EXPECT_NEAR(one_meter_per_second.in(Speed::Unit::FeetPerSecond), kFeetPerSecondPerMeterPerSecond,
              kTolerance);

  EXPECT_NEAR(Speed::meters_per_second(kOneMeterPerSecond).in(Speed::Unit::MetersPerSecond),
              kOneMeterPerSecond, kTolerance);
  EXPECT_NEAR(Speed::kilometers_per_hour(kKilometersPerHourPerMeterPerSecond)
                  .in(Speed::Unit::MetersPerSecond),
              kOneMeterPerSecond, kTolerance);
  EXPECT_NEAR(
      Speed::miles_per_hour(kMilesPerHourPerMeterPerSecond).in(Speed::Unit::MetersPerSecond),
      kOneMeterPerSecond, kTolerance);
  EXPECT_NEAR(
      Speed::feet_per_second(kFeetPerSecondPerMeterPerSecond).in(Speed::Unit::MetersPerSecond),
      kOneMeterPerSecond, kTolerance);
}

TEST(SpeedTest, SupportsRawFactory) {
  const Speed value = Speed::from_raw(12.5);
  EXPECT_DOUBLE_EQ(value.in(Speed::Unit::MetersPerSecond), 12.5);
}

TEST(SpeedTest, HashSpecializationIsUsable) {
  constexpr std::size_t kExpectedSingleEntry = 1U;

  std::unordered_set<Speed> values;
  const Speed sample = Speed::meters_per_second(1.0);
  values.insert(sample);
  values.insert(sample);
  EXPECT_EQ(values.size(), kExpectedSingleEntry);
}

TEST(SpeedTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Speed::Unit>();
  EXPECT_THROW(static_cast<void>(Speed(1.0, invalid_unit)), std::invalid_argument);
}

TEST(SpeedTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Speed::Unit>();
  const Speed speed = Speed::meters_per_second(1.0);
  EXPECT_THROW(static_cast<void>(speed.in(invalid_unit)), std::invalid_argument);
}

} // namespace
