#include <gtest/gtest.h>

#include <memory>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::spherical_geometry::Coordinate;

TEST(CoordinateTest, NormalizesLatitudeAndLongitudeOnConstruction) {
  constexpr double kTolerance = 1e-12;

  const Coordinate coordinate = Coordinate::degrees(120.0, 540.0 + 30.0);

  EXPECT_NEAR(coordinate.latitude().in(Angle::Unit::Degree), 90.0, kTolerance);
  EXPECT_NEAR(coordinate.longitude().in(Angle::Unit::Degree), -150.0, kTolerance);
}

TEST(CoordinateTest, ConvertsToAndFromRadial) {
  constexpr double kTolerance = 1e-12;

  const Coordinate original = Coordinate::degrees(35.0, -73.0);
  const auto radial = original.to_radial();
  const Coordinate round_trip = Coordinate::from_radial(radial);

  EXPECT_NEAR(round_trip.latitude().in(Angle::Unit::Degree), 35.0, kTolerance);
  EXPECT_NEAR(round_trip.longitude().in(Angle::Unit::Degree), -73.0, kTolerance);
}

TEST(CoordinateTest, ProducesExpectedUnitRadial) {
  constexpr double kTolerance = 1e-12;

  const Coordinate coordinate = Coordinate::degrees(0.0, 90.0);
  const auto radial = coordinate.to_radial();

  EXPECT_NEAR(radial.x(), 0.0, kTolerance);
  EXPECT_NEAR(radial.y(), 1.0, kTolerance);
  EXPECT_NEAR(radial.z(), 0.0, kTolerance);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
