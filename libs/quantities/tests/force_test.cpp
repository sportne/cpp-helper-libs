#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "cpp_helper_libs/quantities/force.hpp"
#include "test_helpers.hpp"

namespace {

using cpp_helper_libs::quantities::Force;
using cpp_helper_libs::quantities::test::make_invalid_enum;

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

TEST(ForceTest, HashSpecializationIsUsable) {
  constexpr std::size_t kExpectedSingleEntry = 1U;

  std::unordered_set<Force> values;
  const Force sample = Force::newtons(1.0);
  values.insert(sample);
  values.insert(sample);
  EXPECT_EQ(values.size(), kExpectedSingleEntry);
}

TEST(ForceTest, ConstructorThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Force::Unit>();
  EXPECT_THROW(static_cast<void>(Force(1.0, invalid_unit)), std::invalid_argument);
}

TEST(ForceTest, ConversionThrowsOnInvalidUnit) {
  const auto invalid_unit = make_invalid_enum<Force::Unit>();
  const Force value = Force::newtons(1.0);
  EXPECT_THROW(static_cast<void>(value.in(invalid_unit)), std::invalid_argument);
}

} // namespace
