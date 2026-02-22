#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

#include "cpp_helper_libs/quantities/acceleration.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/quantities/force.hpp"
#include "cpp_helper_libs/quantities/length.hpp"
#include "cpp_helper_libs/quantities/mass.hpp"
#include "cpp_helper_libs/quantities/quantity_base.hpp"
#include "cpp_helper_libs/quantities/speed.hpp"
#include "cpp_helper_libs/quantities/time.hpp"

namespace {

using cpp_helper_libs::quantities::Acceleration;
using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::quantities::Force;
using cpp_helper_libs::quantities::Length;
using cpp_helper_libs::quantities::Mass;
using cpp_helper_libs::quantities::Speed;
using cpp_helper_libs::quantities::Time;

template <typename Enum> Enum make_invalid_enum() {
  Enum value{};
  using Underlying = std::underlying_type_t<Enum>;
  // Use a max underlying value to intentionally leave the defined enum range.
  constexpr Underlying kInvalidRaw = std::numeric_limits<Underlying>::max();
  static_assert(sizeof(Underlying) == sizeof(Enum));
  std::memcpy(&value, &kInvalidRaw, sizeof(Underlying));
  return value;
}

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

TEST(TimeTest, SupportsArithmeticAndFactories) {
  constexpr double kThirtySeconds = 30.0;

  const Time duration = Time::minutes(1.0) + Time::seconds(kThirtySeconds);

  EXPECT_EQ(duration, Time::seconds(90.0));
  EXPECT_EQ(Time::hours(1.0) - Time::minutes(30.0), Time::minutes(30.0));
  EXPECT_DOUBLE_EQ(Time::milliseconds(500.0).in(Time::Unit::Second), 0.5);
}

TEST(TimeTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kNanosecondTolerance = 1e-6;
  constexpr double kOneSecond = 1.0;
  constexpr double kNanosecondsPerSecond = 1000000000.0;
  constexpr double kMicrosecondsPerSecond = 1000000.0;
  constexpr double kMillisecondsPerSecond = 1000.0;
  constexpr double kMinutesPerSecond = 1.0 / 60.0;
  constexpr double kHoursPerSecond = 1.0 / 3600.0;

  const Time one_second = Time::seconds(kOneSecond);

  EXPECT_NEAR(one_second.in(Time::Unit::Nanosecond), kNanosecondsPerSecond, kNanosecondTolerance);
  EXPECT_NEAR(one_second.in(Time::Unit::Microsecond), kMicrosecondsPerSecond, kTolerance);
  EXPECT_NEAR(one_second.in(Time::Unit::Millisecond), kMillisecondsPerSecond, kTolerance);
  EXPECT_NEAR(one_second.in(Time::Unit::Second), kOneSecond, kTolerance);
  EXPECT_NEAR(one_second.in(Time::Unit::Minute), kMinutesPerSecond, kTolerance);
  EXPECT_NEAR(one_second.in(Time::Unit::Hour), kHoursPerSecond, kTolerance);

  EXPECT_NEAR(Time::nanoseconds(kNanosecondsPerSecond).in(Time::Unit::Second), kOneSecond,
              kTolerance);
  EXPECT_NEAR(Time::microseconds(kMicrosecondsPerSecond).in(Time::Unit::Second), kOneSecond,
              kTolerance);
  EXPECT_NEAR(Time::milliseconds(kMillisecondsPerSecond).in(Time::Unit::Second), kOneSecond,
              kTolerance);
  EXPECT_NEAR(Time::seconds(kOneSecond).in(Time::Unit::Second), kOneSecond, kTolerance);
  EXPECT_NEAR(Time::minutes(kMinutesPerSecond).in(Time::Unit::Second), kOneSecond, kTolerance);
  EXPECT_NEAR(Time::hours(kHoursPerSecond).in(Time::Unit::Second), kOneSecond, kTolerance);
}

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

TEST(ForceTest, ConvertsValues) {
  const Force lbf = Force::pound_force(10.0);

  EXPECT_DOUBLE_EQ(lbf.in(Force::Unit::Newton), 44.482216152605);
  EXPECT_EQ(Force::kilonewtons(1.0), Force::newtons(1000.0));
}

TEST(ForceTest, SupportsAllFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneNewton = 1.0;
  constexpr double kKilonewtonsPerNewton = 0.001;
  constexpr double kPoundForcePerNewton = 0.2248089430997105;

  const Force one_newton = Force::newtons(kOneNewton);

  EXPECT_NEAR(one_newton.in(Force::Unit::Newton), kOneNewton, kTolerance);
  EXPECT_NEAR(one_newton.in(Force::Unit::Kilonewton), kKilonewtonsPerNewton, kTolerance);
  EXPECT_NEAR(one_newton.in(Force::Unit::PoundForce), kPoundForcePerNewton, kTolerance);

  EXPECT_NEAR(Force::newtons(kOneNewton).in(Force::Unit::Newton), kOneNewton, kTolerance);
  EXPECT_NEAR(Force::kilonewtons(kKilonewtonsPerNewton).in(Force::Unit::Newton), kOneNewton,
              kTolerance);
  EXPECT_NEAR(Force::pound_force(kPoundForcePerNewton).in(Force::Unit::Newton), kOneNewton,
              kTolerance);
}

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

TEST(AngleTest, ConvertsDegreesAndRadians) {
  const Angle deg_180 = Angle::degrees(180.0);

  EXPECT_NEAR(deg_180.in(Angle::Unit::Radian), 3.14159265358979323846, 1e-12);
  EXPECT_NEAR(Angle::radians(3.14159265358979323846).in(Angle::Unit::Degree), 180.0, 1e-12);
}

TEST(AngleTest, BindsToZeroToTwoPiRange) {
  constexpr double kTolerance = 1e-12;
  constexpr double kZero = 0.0;
  constexpr double kTenPi = 10.0 * 180.0;
  constexpr double kNegativeTenPi = -10.0 * 180.0;

  const Angle positive = Angle::degrees(450.0).bound_zero_to_two_pi();
  const Angle negative = Angle::degrees(-90.0).bound_zero_to_two_pi();
  const Angle positive_many_turns = Angle::degrees(kTenPi).bound_zero_to_two_pi();
  const Angle negative_many_turns = Angle::degrees(kNegativeTenPi).bound_zero_to_two_pi();

  EXPECT_NEAR(positive.in(Angle::Unit::Degree), 90.0, kTolerance);
  EXPECT_NEAR(negative.in(Angle::Unit::Degree), 270.0, kTolerance);
  EXPECT_NEAR(positive_many_turns.in(Angle::Unit::Radian), kZero, kTolerance);
  EXPECT_NEAR(negative_many_turns.in(Angle::Unit::Radian), kZero, kTolerance);
}

TEST(AngleTest, BindsToNegativePiToPiRange) {
  constexpr double kTolerance = 1e-12;
  constexpr double kZero = 0.0;
  constexpr double kTenPi = 10.0 * 180.0;
  constexpr double kNegativeTenPi = -10.0 * 180.0;

  const Angle wrapped = Angle::degrees(270.0).bound_negative_pi_to_pi();
  const Angle boundary = Angle::degrees(180.0).bound_negative_pi_to_pi();
  const Angle positive_many_turns = Angle::degrees(kTenPi).bound_negative_pi_to_pi();
  const Angle negative_many_turns = Angle::degrees(kNegativeTenPi).bound_negative_pi_to_pi();

  EXPECT_NEAR(wrapped.in(Angle::Unit::Degree), -90.0, kTolerance);
  EXPECT_NEAR(boundary.in(Angle::Unit::Degree), -180.0, kTolerance);
  EXPECT_NEAR(positive_many_turns.in(Angle::Unit::Radian), kZero, kTolerance);
  EXPECT_NEAR(negative_many_turns.in(Angle::Unit::Radian), kZero, kTolerance);
}

TEST(AngleTest, SupportsFactoriesAndUnitConversions) {
  constexpr double kTolerance = 1e-12;
  constexpr double kOneRadian = 1.0;
  constexpr double kDegreesPerRadian = 57.29577951308232;

  const Angle one_radian = Angle::radians(kOneRadian);

  EXPECT_NEAR(one_radian.in(Angle::Unit::Radian), kOneRadian, kTolerance);
  EXPECT_NEAR(one_radian.in(Angle::Unit::Degree), kDegreesPerRadian, kTolerance);
  EXPECT_NEAR(Angle::degrees(kDegreesPerRadian).in(Angle::Unit::Radian), kOneRadian, kTolerance);
}

TEST(HashTest, EqualQuantitiesProduceEqualHashes) {
  const Length meters = Length::meters(1000.0);
  const Length kilometers = Length::kilometers(1.0);

  EXPECT_EQ(meters.hash_code(), kilometers.hash_code());

  std::unordered_set<Length> values;
  values.insert(meters);
  values.insert(kilometers);

  EXPECT_EQ(values.size(), 1U);
}

TEST(HashTest, PositiveAndNegativeZeroHashIdentically) {
  const Time positive_zero = Time::seconds(0.0);
  const Time negative_zero = Time::seconds(-0.0);

  EXPECT_EQ(positive_zero, negative_zero);
  EXPECT_EQ(positive_zero.hash_code(), negative_zero.hash_code());
}

TEST(HashTest, HashSpecializationsAreUsableForAllQuantities) {
  constexpr std::size_t kExpectedSingleEntry = 1U;

  std::unordered_set<Mass> mass_values;
  const Mass mass_sample = Mass::kilograms(1.0);
  mass_values.insert(mass_sample);
  mass_values.insert(mass_sample);
  EXPECT_EQ(mass_values.size(), kExpectedSingleEntry);

  std::unordered_set<Force> force_values;
  const Force force_sample = Force::newtons(1.0);
  force_values.insert(force_sample);
  force_values.insert(force_sample);
  EXPECT_EQ(force_values.size(), kExpectedSingleEntry);

  std::unordered_set<Speed> speed_values;
  const Speed speed_sample = Speed::meters_per_second(1.0);
  speed_values.insert(speed_sample);
  speed_values.insert(speed_sample);
  EXPECT_EQ(speed_values.size(), kExpectedSingleEntry);

  std::unordered_set<Acceleration> acceleration_values;
  const Acceleration acceleration_sample = Acceleration::meters_per_second_squared(1.0);
  acceleration_values.insert(acceleration_sample);
  acceleration_values.insert(acceleration_sample);
  EXPECT_EQ(acceleration_values.size(), kExpectedSingleEntry);
}

TEST(ScalingTest, SupportsMultiplyAndDivide) {
  constexpr double kScaleUp = 3.0;
  constexpr double kScaleDown = 2.0;

  const Length base = Length::meters(2.0);
  const Length scaled_up = base.multiplied_by(kScaleUp);
  const Length scaled_down = scaled_up.divided_by(kScaleDown);

  EXPECT_EQ(scaled_up, Length::meters(6.0));
  EXPECT_EQ(scaled_down, Length::meters(3.0));
}

TEST(ScalingTest, SupportsInPlaceScaling) {
  constexpr double kInitialSeconds = 2.0;
  constexpr double kScaleUp = 4.0;
  constexpr double kScaleDown = 8.0;

  const Time duration =
      Time::seconds(kInitialSeconds).multiplied_by(kScaleUp).divided_by(kScaleDown);

  EXPECT_EQ(duration, Time::seconds(1.0));
}

TEST(ScalingTest, ThrowsOnDivideByZero) {
  constexpr double kZero = 0.0;

  const Mass mass = Mass::kilograms(5.0);
  EXPECT_THROW(static_cast<void>(mass.divided_by(kZero)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidLengthUnit) {
  const auto invalid_unit = make_invalid_enum<Length::Unit>();
  EXPECT_THROW(static_cast<void>(Length(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidTimeUnit) {
  const auto invalid_unit = make_invalid_enum<Time::Unit>();
  EXPECT_THROW(static_cast<void>(Time(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidMassUnit) {
  const auto invalid_unit = make_invalid_enum<Mass::Unit>();
  EXPECT_THROW(static_cast<void>(Mass(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidForceUnit) {
  const auto invalid_unit = make_invalid_enum<Force::Unit>();
  EXPECT_THROW(static_cast<void>(Force(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidSpeedUnit) {
  const auto invalid_unit = make_invalid_enum<Speed::Unit>();
  EXPECT_THROW(static_cast<void>(Speed(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidAccelerationUnit) {
  const auto invalid_unit = make_invalid_enum<Acceleration::Unit>();
  EXPECT_THROW(static_cast<void>(Acceleration(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConstructorThrowsOnInvalidAngleUnit) {
  const auto invalid_unit = make_invalid_enum<Angle::Unit>();
  EXPECT_THROW(static_cast<void>(Angle(1.0, invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidLengthUnit) {
  const auto invalid_unit = make_invalid_enum<Length::Unit>();
  const Length value = Length::meters(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidTimeUnit) {
  const auto invalid_unit = make_invalid_enum<Time::Unit>();
  const Time value = Time::seconds(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidMassUnit) {
  const auto invalid_unit = make_invalid_enum<Mass::Unit>();
  const Mass value = Mass::kilograms(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidForceUnit) {
  const auto invalid_unit = make_invalid_enum<Force::Unit>();
  const Force value = Force::newtons(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidSpeedUnit) {
  const auto invalid_unit = make_invalid_enum<Speed::Unit>();
  const Speed speed = Speed::meters_per_second(1.0);

  EXPECT_THROW(static_cast<void>(speed.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidAccelerationUnit) {
  const auto invalid_unit = make_invalid_enum<Acceleration::Unit>();
  const Acceleration value = Acceleration::meters_per_second_squared(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

TEST(InvalidUnitTest, ConversionThrowsOnInvalidAngleUnit) {
  const auto invalid_unit = make_invalid_enum<Angle::Unit>();
  const Angle value = Angle::radians(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
