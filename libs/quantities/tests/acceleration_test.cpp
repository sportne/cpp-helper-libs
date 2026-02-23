#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "cpp_helper_libs/quantities/acceleration.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Acceleration;
using cpp_helper_libs::quantities::test::make_invalid_enum;

TEST(AccelerationTest, HandlesStandardGravity) {
  const Acceleration gravity_acceleration = Acceleration::standard_gravity(1.0);
  const Acceleration feet = Acceleration::feet_per_second_squared(32.1740485564304);

  EXPECT_DOUBLE_EQ(gravity_acceleration.in(Acceleration::Unit::MetersPerSecondSquared), 9.80665);
  EXPECT_NEAR(feet.in(Acceleration::Unit::MetersPerSecondSquared), 9.80665, 1e-12);
}

TEST(AccelerationTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneMeterPerSecondSquared = 1.0;
  constexpr double kFeetPerSecondSquaredPerMeterPerSecondSquared = 3.280839895013123;
  constexpr double kStandardGravityPerMeterPerSecondSquared = 0.10197162129779283;

  const Acceleration one_meter_per_second_squared =
      Acceleration::meters_per_second_squared(kOneMeterPerSecondSquared);

  EXPECT_NEAR(one_meter_per_second_squared.in(Acceleration::Unit::MetersPerSecondSquared),
              kOneMeterPerSecondSquared, kTolerance);
  EXPECT_NEAR(one_meter_per_second_squared.in(Acceleration::Unit::FeetPerSecondSquared),
              kFeetPerSecondSquaredPerMeterPerSecondSquared, kTolerance);
  EXPECT_NEAR(one_meter_per_second_squared.in(Acceleration::Unit::StandardGravity),
              kStandardGravityPerMeterPerSecondSquared, kTolerance);

  EXPECT_NEAR(Acceleration::meters_per_second_squared(kOneMeterPerSecondSquared)
                  .in(Acceleration::Unit::MetersPerSecondSquared),
              kOneMeterPerSecondSquared, kTolerance);
  EXPECT_NEAR(Acceleration::feet_per_second_squared(kFeetPerSecondSquaredPerMeterPerSecondSquared)
                  .in(Acceleration::Unit::MetersPerSecondSquared),
              kOneMeterPerSecondSquared, kTolerance);
  EXPECT_NEAR(Acceleration::standard_gravity(kStandardGravityPerMeterPerSecondSquared)
                  .in(Acceleration::Unit::MetersPerSecondSquared),
              kOneMeterPerSecondSquared, kTolerance);
}

TEST(AccelerationTest, SupportsRawFactory) {
  const Acceleration value = Acceleration::from_raw(9.80665);
  EXPECT_DOUBLE_EQ(value.in(Acceleration::Unit::MetersPerSecondSquared), 9.80665);
}

TEST(AccelerationTest, HashSpecializationIsUsable) {
  constexpr std::size_t kExpectedSingleEntry = 1U;

  std::unordered_set<Acceleration> values;
  const Acceleration sample = Acceleration::meters_per_second_squared(1.0);
  values.insert(sample);
  values.insert(sample);
  EXPECT_EQ(values.size(), kExpectedSingleEntry);
}

TEST(AccelerationTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Acceleration::Unit>();
  EXPECT_THROW(static_cast<void>(Acceleration(1.0, invalid_unit)), std::invalid_argument);
}

TEST(AccelerationTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Acceleration::Unit>();
  const Acceleration value = Acceleration::meters_per_second_squared(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
