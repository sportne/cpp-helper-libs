#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <stdexcept>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return candidate.value();
}

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::spherical_geometry::SphericalRay;

TEST(SphericalRayTest, BuildsFromRadialAndTangent) {
  const UnitVector3 radial = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));

  const std::optional<SphericalRay> ray = SphericalRay::from_radial_and_tangent(radial, tangent);

  const SphericalRay ray_value = require_value(ray);
  EXPECT_DOUBLE_EQ(ray_value.normal().z(), 1.0);
}

TEST(SphericalRayTest, ProjectsForwardAlongGreatCircle) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 radial = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay ray = require_value(SphericalRay::from_radial_and_tangent(radial, tangent));

  const SphericalRay advanced = ray.project_forward(Angle::degrees(90.0));

  EXPECT_NEAR(advanced.radial().x(), 0.0, kTolerance);
  EXPECT_NEAR(advanced.radial().y(), 1.0, kTolerance);
  EXPECT_NEAR(advanced.radial().z(), 0.0, kTolerance);
}

TEST(SphericalRayTest, RotatesAroundRadial) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 radial = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const SphericalRay ray = require_value(SphericalRay::from_radial_and_tangent(radial, tangent));

  const SphericalRay rotated = ray.rotate_about_radial(Angle::degrees(90.0));

  EXPECT_NEAR(rotated.tangent().x(), 0.0, kTolerance);
  EXPECT_NEAR(rotated.tangent().y(), 0.0, kTolerance);
  EXPECT_NEAR(rotated.tangent().z(), 1.0, kTolerance);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
