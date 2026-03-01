// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "cpp_helper_libs/dubins_path_finding/spherical_dubins_planner.hpp"
#include "cpp_helper_libs/linear_algebra/matrix3.hpp"
#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/math/arithmetic.hpp"
#include "cpp_helper_libs/path_finding/a_star.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/quantities/length.hpp"
#include "cpp_helper_libs/quantities/speed.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace {

constexpr std::uint64_t kSmokeEdgePayload = 99U;
constexpr double kSmokeMinimumTurnRadiusDegrees = 20.0;
constexpr double kEarthRadiusMeters = 6371000.0;

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return candidate.value();
}

} // namespace

TEST(ProjectSmokeSharedTest, MathSharedLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(3, 4), 7);
}

TEST(ProjectSmokeSharedTest, QuantitiesSharedLibraryLinksAndExecutes) {
  const auto speed = cpp_helper_libs::quantities::Speed::kilometers_per_hour(72.0);
  const auto angle = cpp_helper_libs::quantities::Angle::degrees(180.0);

  EXPECT_EQ(speed, cpp_helper_libs::quantities::Speed::meters_per_second(20.0));
  EXPECT_NEAR(angle.in(cpp_helper_libs::quantities::Angle::Unit::Radian), 3.14159265358979323846,
              1e-12);
}

TEST(ProjectSmokeSharedTest, LinearAlgebraSharedLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const auto tangent = cpp_helper_libs::linear_algebra::unit_tangent(
      cpp_helper_libs::linear_algebra::Vector3(0.0, 3.0, 4.0));
  const auto x_axis = cpp_helper_libs::linear_algebra::UnitVector3::from_components(1.0, 0.0, 0.0);
  const auto y_axis = cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 1.0, 0.0);

  const cpp_helper_libs::linear_algebra::UnitVector3 tangent_value = require_value(tangent);
  const cpp_helper_libs::linear_algebra::UnitVector3 x_axis_value = require_value(x_axis);
  const cpp_helper_libs::linear_algebra::UnitVector3 y_axis_value = require_value(y_axis);
  EXPECT_NEAR(tangent_value.z(), 0.8, kTolerance);
  EXPECT_NEAR(x_axis_value.central_angle_radians(y_axis_value), std::numbers::pi_v<double> / 2.0,
              kTolerance);
}

TEST(ProjectSmokeSharedTest, Matrix3SharedLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const cpp_helper_libs::linear_algebra::Matrix3 matrix_value(1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0,
                                                              6.0, 0.0);
  const cpp_helper_libs::linear_algebra::Matrix3 product = matrix_value.multiply(matrix_value);

  EXPECT_NEAR(product(0U, 0U), 16.0, kTolerance);
  EXPECT_NEAR(product(0U, 1U), 22.0, kTolerance);
  EXPECT_NEAR(product(0U, 2U), 11.0, kTolerance);
}

TEST(ProjectSmokeSharedTest, SphericalGeometrySharedLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const auto coordinate = cpp_helper_libs::spherical_geometry::Coordinate::degrees(30.0, -45.0);
  const auto radial = coordinate.to_radial();
  const auto round_trip = cpp_helper_libs::spherical_geometry::Coordinate::from_radial(radial);

  EXPECT_NEAR(round_trip.latitude().in(cpp_helper_libs::quantities::Angle::Unit::Degree), 30.0,
              kTolerance);
  EXPECT_NEAR(round_trip.longitude().in(cpp_helper_libs::quantities::Angle::Unit::Degree), -45.0,
              kTolerance);
}

TEST(ProjectSmokeSharedTest, PathFindingSharedLibraryLinksAndExecutes) {
  class OneStepProblem final : public cpp_helper_libs::path_finding::AStarProblem {
  public:
    [[nodiscard]] bool
    is_goal(const cpp_helper_libs::path_finding::NodeId node) const noexcept override {
      return node == 2U;
    }

    [[nodiscard]] double
    heuristic(const cpp_helper_libs::path_finding::NodeId node) const noexcept override {
      return node == 2U ? 0.0 : 1.0;
    }

    void expand(const cpp_helper_libs::path_finding::NodeId node,
                std::vector<cpp_helper_libs::path_finding::WeightedEdge> *out_edges) override {
      out_edges->clear();
      if (node == 1U) {
        out_edges->push_back(cpp_helper_libs::path_finding::WeightedEdge{
            .to = 2U, .cost = 1.0, .payload = kSmokeEdgePayload});
      }
    }
  };

  OneStepProblem problem;
  const cpp_helper_libs::path_finding::AStarResult result =
      cpp_helper_libs::path_finding::solve_a_star(problem, 1U);
  EXPECT_EQ(result.status, cpp_helper_libs::path_finding::AStarStatus::Found);
}

TEST(ProjectSmokeSharedTest, DubinsPathFindingSharedLibraryLinksAndExecutes) {
  const cpp_helper_libs::spherical_geometry::SphericalRay start =
      require_value(cpp_helper_libs::spherical_geometry::SphericalRay::from_radial_and_tangent(
          cpp_helper_libs::spherical_geometry::Coordinate::degrees(0.0, 0.0).to_radial(),
          cpp_helper_libs::spherical_geometry::Coordinate::degrees(0.0, 90.0).to_radial()));
  const cpp_helper_libs::spherical_geometry::SphericalRay goal =
      require_value(cpp_helper_libs::spherical_geometry::SphericalRay::from_radial_and_tangent(
          cpp_helper_libs::spherical_geometry::Coordinate::degrees(0.0, 5.0).to_radial(),
          cpp_helper_libs::spherical_geometry::Coordinate::degrees(0.0, 95.0).to_radial()));

  cpp_helper_libs::dubins_path_finding::Request request{
      .start = start,
      .goal = goal,
      .environment = cpp_helper_libs::dubins_path_finding::SphericalEnvironment{},
      .vehicle =
          cpp_helper_libs::dubins_path_finding::DubinsVehicleModel{
              .minimum_turn_radius =
                  cpp_helper_libs::quantities::Angle::degrees(kSmokeMinimumTurnRadiusDegrees)},
      .sphere_radius = cpp_helper_libs::quantities::Length::meters(kEarthRadiusMeters),
      .config = cpp_helper_libs::dubins_path_finding::PlannerConfig{},
  };

  const cpp_helper_libs::dubins_path_finding::Result result =
      cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);
  EXPECT_NE(result.status, cpp_helper_libs::dubins_path_finding::PlannerStatus::InvalidInput);
}
