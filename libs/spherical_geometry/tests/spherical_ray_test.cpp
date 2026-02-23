#include <gtest/gtest.h>

#include <cmath>
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

TEST(SphericalRayTest, ValidatesFrameOrthogonalityAndOrientation) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 y_axis = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 z_axis = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));
  const UnitVector3 negative_z_axis = require_value(UnitVector3::from_components(0.0, 0.0, -1.0));

  EXPECT_FALSE(SphericalRay::from_frame(x_axis, x_axis, y_axis).has_value());
  EXPECT_FALSE(SphericalRay::from_frame(x_axis, z_axis, x_axis).has_value());
  EXPECT_FALSE(SphericalRay::from_frame(x_axis, y_axis, y_axis).has_value());
  EXPECT_FALSE(SphericalRay::from_frame(x_axis, negative_z_axis, y_axis).has_value());
}

TEST(SphericalRayTest, RejectsNonOrthogonalRadialAndTangent) {
  const UnitVector3 x_axis = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 diagonal =
      require_value(UnitVector3::from_components(std::sqrt(0.5), std::sqrt(0.5), 0.0));

  EXPECT_FALSE(SphericalRay::from_radial_and_tangent(x_axis, diagonal).has_value());
}

TEST(SphericalRayTest, ExactProjectionAndRotationMatchExpectedFrames) {
  constexpr double kTolerance = 1e-12;

  const UnitVector3 radial = require_value(UnitVector3::from_components(1.0, 0.0, 0.0));
  const UnitVector3 tangent = require_value(UnitVector3::from_components(0.0, 1.0, 0.0));
  const UnitVector3 normal = require_value(UnitVector3::from_components(0.0, 0.0, 1.0));
  const SphericalRay ray = require_value(SphericalRay::from_frame(radial, normal, tangent));

  const SphericalRay projected = ray.project_forward_exact(Angle::degrees(90.0));
  EXPECT_NEAR(projected.radial().x(), 0.0, kTolerance);
  EXPECT_NEAR(projected.radial().y(), 1.0, kTolerance);
  EXPECT_NEAR(projected.normal().z(), 1.0, kTolerance);

  const SphericalRay rotated = ray.rotate_about_radial_exact(Angle::degrees(90.0));
  EXPECT_NEAR(rotated.normal().x(), 0.0, kTolerance);
  EXPECT_NEAR(rotated.normal().y(), -1.0, kTolerance);
  EXPECT_NEAR(rotated.normal().z(), 0.0, kTolerance);
  EXPECT_NEAR(rotated.tangent().z(), 1.0, kTolerance);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
