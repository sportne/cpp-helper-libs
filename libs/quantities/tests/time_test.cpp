#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "cpp_helper_libs/quantities/quantity_base.hpp"
#include "cpp_helper_libs/quantities/time.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Time;
using cpp_helper_libs::quantities::test::make_invalid_enum;

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

TEST(TimeTest, PositiveAndNegativeZeroHashIdentically) {
  const Time positive_zero = Time::seconds(0.0);
  const Time negative_zero = Time::seconds(-0.0);

  EXPECT_EQ(positive_zero, negative_zero);
  EXPECT_EQ(positive_zero.hash_code(), negative_zero.hash_code());
}

TEST(TimeTest, SupportsInPlaceScaling) {
  constexpr double kInitialSeconds = 2.0;
  constexpr double kScaleUp = 4.0;
  constexpr double kScaleDown = 8.0;

  const Time duration =
      Time::seconds(kInitialSeconds).multiplied_by(kScaleUp).divided_by(kScaleDown);

  EXPECT_EQ(duration, Time::seconds(1.0));
}

TEST(TimeTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Time::Unit>();
  EXPECT_THROW(static_cast<void>(Time(1.0, invalid_unit)), std::invalid_argument);
}

TEST(TimeTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Time::Unit>();
  const Time value = Time::seconds(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
