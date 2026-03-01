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
#include "cpp_helper_libs/quantities/quantity_base.hpp"
#include "cpp_helper_libs/quantities/time.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace {

constexpr std::uint64_t kSmokeEdgePayload = 42U;
constexpr double kSmokeMinimumTurnRadiusDegrees = 20.0;
constexpr double kEarthRadiusMeters = 6371000.0;

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return *candidate;
}

} // namespace

TEST(ProjectSmokeTest, MathLibraryLinksAndExecutes) {
  EXPECT_EQ(cpp_helper_libs::math::add(1, 2), 3);
}

TEST(ProjectSmokeTest, QuantitiesLibraryLinksAndExecutes) {
  const auto length = cpp_helper_libs::quantities::Length::meters(2.0);
  const auto time = cpp_helper_libs::quantities::Time::seconds(4.0);

  EXPECT_EQ(length + cpp_helper_libs::quantities::Length::meters(3.0),
            cpp_helper_libs::quantities::Length::meters(5.0));
  EXPECT_DOUBLE_EQ(time.in(cpp_helper_libs::quantities::Time::Unit::Millisecond), 4000.0);
}

TEST(ProjectSmokeTest, LinearAlgebraLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const cpp_helper_libs::linear_algebra::Vector3 x_axis(1.0, 0.0, 0.0);
  const cpp_helper_libs::linear_algebra::Vector3 y_axis(0.0, 1.0, 0.0);
  const auto right_normal = cpp_helper_libs::linear_algebra::unit_normal(x_axis, y_axis);
  const auto angle = x_axis.central_angle_radians(y_axis);

  const cpp_helper_libs::linear_algebra::UnitVector3 right_normal_value =
      require_value(right_normal);
  const double angle_value = require_value(angle);
  EXPECT_EQ(right_normal_value.as_vector(),
            cpp_helper_libs::linear_algebra::Vector3(0.0, 0.0, 1.0));
  EXPECT_NEAR(angle_value, std::numbers::pi_v<double> / 2.0, kTolerance);
}

TEST(ProjectSmokeTest, Matrix3LibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const cpp_helper_libs::linear_algebra::Matrix3 matrix_value(4.0, 1.0, 2.0, 1.0, 5.0, 1.0, 2.0,
                                                              1.0, 3.0);
  const cpp_helper_libs::linear_algebra::Vector3 rhs(7.0, 8.0, 5.0);

  const auto solved = matrix_value.solve(rhs);
  const cpp_helper_libs::linear_algebra::Vector3 solved_value = require_value(solved);
  EXPECT_NEAR(solved_value.x(), 1.2162162162162162, kTolerance);
  EXPECT_NEAR(matrix_value.determinant(), 37.0, kTolerance);
}

TEST(ProjectSmokeTest, SphericalGeometryLibraryLinksAndExecutes) {
  constexpr double kTolerance = 1e-12;

  const auto x_axis =
      require_value(cpp_helper_libs::linear_algebra::UnitVector3::from_components(1.0, 0.0, 0.0));
  const auto y_axis =
      require_value(cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 1.0, 0.0));

  const auto minor_arc =
      require_value(cpp_helper_libs::spherical_geometry::MinorArc::from_endpoints(x_axis, y_axis));

  const auto coordinate =
      cpp_helper_libs::spherical_geometry::Coordinate::from_radial(minor_arc.end_radial());
  EXPECT_NEAR(coordinate.longitude().in(cpp_helper_libs::quantities::Angle::Unit::Degree), 90.0,
              kTolerance);
}

TEST(ProjectSmokeTest, PathFindingLibraryLinksAndExecutes) {
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
  EXPECT_EQ(result.node_path, (std::vector<cpp_helper_libs::path_finding::NodeId>{1U, 2U}));
}

TEST(ProjectSmokeTest, DubinsPathFindingLibraryLinksAndExecutes) {
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
