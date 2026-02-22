#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "cpp_helper_libs/quantities/angle.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::quantities::test::make_invalid_enum;

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

TEST(AngleTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Angle::Unit>();
  EXPECT_THROW(static_cast<void>(Angle(1.0, invalid_unit)), std::invalid_argument);
}

TEST(AngleTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Angle::Unit>();
  const Angle value = Angle::radians(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
