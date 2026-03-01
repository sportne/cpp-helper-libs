#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <variant>
#include <vector>

#include "cpp_helper_libs/dubins_path_finding/spherical_dubins_planner.hpp"
#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/quantities/length.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_circle.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_polygon.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
namespace {

template <typename ValueType> ValueType require_value(const std::optional<ValueType> &candidate) {
  if (!candidate.has_value()) {
    throw std::runtime_error("Expected optional to contain a value");
  }

  return *candidate;
}

using cpp_helper_libs::dubins_path_finding::DubinsVehicleModel;
using cpp_helper_libs::dubins_path_finding::ObstructionArea;
using cpp_helper_libs::dubins_path_finding::PathSegment;
using cpp_helper_libs::dubins_path_finding::PlannerConfig;
using cpp_helper_libs::dubins_path_finding::PlannerStatus;
using cpp_helper_libs::dubins_path_finding::Request;
using cpp_helper_libs::dubins_path_finding::Result;
using cpp_helper_libs::dubins_path_finding::SphericalEnvironment;
using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::linear_algebra::Vector3;
using cpp_helper_libs::quantities::Angle;
using cpp_helper_libs::quantities::Length;
using cpp_helper_libs::spherical_geometry::Coordinate;
using cpp_helper_libs::spherical_geometry::MinorArc;
using cpp_helper_libs::spherical_geometry::SmallArc;
using cpp_helper_libs::spherical_geometry::SphericalCircle;
using cpp_helper_libs::spherical_geometry::SphericalCurve;
using cpp_helper_libs::spherical_geometry::SphericalPolygon;
using cpp_helper_libs::spherical_geometry::SphericalRay;
using cpp_helper_libs::spherical_geometry::TurnDirection;

struct RaySpec final {
  double latitude_degrees;
  double longitude_degrees;
  double heading_degrees_clockwise_from_north;
};

SphericalRay make_ray(const RaySpec &spec) {
  const Coordinate coordinate = Coordinate::degrees(spec.latitude_degrees, spec.longitude_degrees);
  const UnitVector3 radial = coordinate.to_radial();

  const double longitude_radians = coordinate.longitude().in(Angle::Unit::Radian);
  const Vector3 east_guess(-std::sin(longitude_radians), std::cos(longitude_radians), 0.0);
  const UnitVector3 east = require_value(UnitVector3::from_vector(east_guess));
  const UnitVector3 north = require_value(UnitVector3::from_vector(radial.cross(east)));

  const double heading_radians =
      Angle::degrees(spec.heading_degrees_clockwise_from_north).in(Angle::Unit::Radian);
  const Vector3 tangent_guess =
      north.scaled_by(std::cos(heading_radians)) + east.scaled_by(std::sin(heading_radians));
  const UnitVector3 tangent = require_value(UnitVector3::from_vector(tangent_guess));

  return require_value(SphericalRay::from_radial_and_tangent(radial, tangent));
}

PlannerConfig default_config() {
  return PlannerConfig{
      .lattice_position_resolution = Angle::degrees(1.0),
      .heading_bin_count = 72U,
      .forward_step = Angle::degrees(1.0),
      .turn_step = Angle::degrees(3.0),
      .goal_position_tolerance = Angle::degrees(6.0),
      .goal_heading_tolerance = Angle::degrees(45.0),
      .max_expanded_nodes = 80000U,
  };
}

Request base_request() {
  Request request{
      .start = make_ray(RaySpec{.latitude_degrees = 0.0,
                                .longitude_degrees = 0.0,
                                .heading_degrees_clockwise_from_north = 90.0}),
      .goal = make_ray(RaySpec{.latitude_degrees = 0.0,
                               .longitude_degrees = 16.0,
                               .heading_degrees_clockwise_from_north = 90.0}),
      .environment = SphericalEnvironment{},
      .vehicle = DubinsVehicleModel{.minimum_turn_radius = Angle::degrees(10.0)},
      .sphere_radius = Length::meters(6371000.0),
      .config = default_config(),
  };
  return request;
}

void expect_path_is_well_formed(const cpp_helper_libs::dubins_path_finding::Path &path) {
  EXPECT_FALSE(path.segments.empty());
  EXPECT_GT(path.angular_length.in(Angle::Unit::Radian), 0.0);
  EXPECT_GT(path.surface_length.in(Length::Unit::Meter), 0.0);

  for (const PathSegment &segment : path.segments) {
    const bool is_minor_or_small =
        std::holds_alternative<MinorArc>(segment) || std::holds_alternative<SmallArc>(segment);
    EXPECT_TRUE(is_minor_or_small);
  }
}

bool segment_is_clear(const PathSegment &segment,
                      const std::vector<ObstructionArea> &obstructions) {
  return std::visit(
      [&](const auto &segment_value) {
        const SphericalCurve &curve = segment_value;
        const UnitVector3 start = curve.start_radial();
        const UnitVector3 end = curve.end_radial();
        const UnitVector3 midpoint = curve.point_at_parameter(0.5);

        for (const ObstructionArea &obstruction : obstructions) {
          const bool contains_any_point = std::visit(
              [&](const auto &shape) {
                return shape.contains_inclusive_exact(start) ||
                       shape.contains_inclusive_exact(end) ||
                       shape.contains_inclusive_exact(midpoint);
              },
              obstruction);

          if (contains_any_point) {
            return false;
          }

          const bool intersects_boundary = std::visit(
              [&](const auto &shape) { return shape.boundary_intersects_inclusive_exact(curve); },
              obstruction);

          if (intersects_boundary) {
            return false;
          }
        }

        return true;
      },
      segment);
}

std::optional<SmallArc> build_turn_segment_for_test(const SphericalRay &start_ray,
                                                    const Angle turn_radius, const Angle sweep,
                                                    const TurnDirection direction) {
  const double turn_radius_radians = turn_radius.in(Angle::Unit::Radian);
  const double sign = direction == TurnDirection::CounterClockwise ? 1.0 : -1.0;

  const Vector3 center_guess = start_ray.radial().scaled_by(std::cos(turn_radius_radians)) +
                               start_ray.normal().scaled_by(sign * std::sin(turn_radius_radians));
  const std::optional<UnitVector3> center = UnitVector3::from_vector(center_guess);
  if (!center.has_value()) {
    return std::nullopt;
  }

  return SmallArc::from_center_start_direction_and_sweep(center.value(), turn_radius, start_ray,
                                                         direction, sweep);
}

TEST(SphericalDubinsPlannerTest, RejectsInvalidPlannerConfig) {
  Request request = base_request();
  request.config.max_expanded_nodes = 0U;

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  EXPECT_EQ(result.status, PlannerStatus::InvalidInput);
  EXPECT_FALSE(result.path.has_value());
}

TEST(SphericalDubinsPlannerTest, RejectsRequestsOutsideFiftyDegreeEnvelope) {
  const Request base = base_request();
  const Request request{
      .start = base.start,
      .goal = make_ray(RaySpec{.latitude_degrees = 0.0,
                               .longitude_degrees = 70.0,
                               .heading_degrees_clockwise_from_north = 90.0}),
      .environment = base.environment,
      .vehicle = base.vehicle,
      .sphere_radius = base.sphere_radius,
      .config = base.config,
  };

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  EXPECT_EQ(result.status, PlannerStatus::InvalidInput);
  EXPECT_FALSE(result.path.has_value());
}

TEST(SphericalDubinsPlannerTest, RejectsStartInsideObstacle) {
  Request request = base_request();

  const SphericalCircle start_cover = require_value(
      SphericalCircle::from_center_and_radius(request.start.radial(), Angle::degrees(8.0)));
  request.environment.obstructions.emplace_back(start_cover);

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  EXPECT_EQ(result.status, PlannerStatus::InvalidInput);
  EXPECT_FALSE(result.path.has_value());
}

TEST(SphericalDubinsPlannerTest, FindsPathInOpenEnvironment) {
  Request request = base_request();

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  ASSERT_EQ(result.status, PlannerStatus::Found);
  ASSERT_TRUE(result.path.has_value());
  const cpp_helper_libs::dubins_path_finding::Path path = require_value(result.path);
  expect_path_is_well_formed(path);
}

TEST(SphericalDubinsPlannerTest, FindsDetourAroundCircleObstacle) {
  Request request = base_request();

  const SphericalCircle obstacle = require_value(SphericalCircle::from_center_and_radius(
      Coordinate::degrees(2.5, 8.0).to_radial(), Angle::degrees(1.8)));
  request.environment.obstructions.emplace_back(obstacle);

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  ASSERT_EQ(result.status, PlannerStatus::Found);
  ASSERT_TRUE(result.path.has_value());
  const cpp_helper_libs::dubins_path_finding::Path path = require_value(result.path);

  for (const PathSegment &segment : path.segments) {
    EXPECT_TRUE(segment_is_clear(segment, request.environment.obstructions));
  }
}

TEST(SphericalDubinsPlannerTest, FindsDetourAroundPolygonObstacle) {
  Request request = base_request();

  const SphericalPolygon obstacle = require_value(SphericalPolygon::from_vertices(
      {Coordinate::degrees(2.0, 7.0).to_radial(), Coordinate::degrees(2.0, 9.0).to_radial(),
       Coordinate::degrees(-2.0, 9.0).to_radial(), Coordinate::degrees(-2.0, 7.0).to_radial()}));
  request.environment.obstructions.emplace_back(obstacle);

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  ASSERT_EQ(result.status, PlannerStatus::Found);
  ASSERT_TRUE(result.path.has_value());
  const cpp_helper_libs::dubins_path_finding::Path path = require_value(result.path);

  for (const PathSegment &segment : path.segments) {
    EXPECT_TRUE(segment_is_clear(segment, request.environment.obstructions));
  }
}

TEST(SphericalDubinsPlannerTest, ReturnsNoPathWhenAllInitialMovesAreBlocked) {
  Request request = base_request();

  const MinorArc forward =
      require_value(MinorArc::from_start_and_sweep(request.start, request.config.forward_step));
  const SmallArc left = require_value(
      build_turn_segment_for_test(request.start, request.vehicle.minimum_turn_radius,
                                  request.config.turn_step, TurnDirection::CounterClockwise));
  const SmallArc right = require_value(
      build_turn_segment_for_test(request.start, request.vehicle.minimum_turn_radius,
                                  request.config.turn_step, TurnDirection::Clockwise));

  request.environment.obstructions.emplace_back(require_value(
      SphericalCircle::from_center_and_radius(forward.end_radial(), Angle::degrees(0.2))));
  request.environment.obstructions.emplace_back(require_value(
      SphericalCircle::from_center_and_radius(left.end_radial(), Angle::degrees(0.2))));
  request.environment.obstructions.emplace_back(require_value(
      SphericalCircle::from_center_and_radius(right.end_radial(), Angle::degrees(0.2))));

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  EXPECT_EQ(result.status, PlannerStatus::NoPath);
  EXPECT_FALSE(result.path.has_value());
}

TEST(SphericalDubinsPlannerTest, ReturnsSearchLimitReachedWhenBudgetIsTooSmall) {
  Request request = base_request();
  request.config.max_expanded_nodes = 1U;

  const Result result = cpp_helper_libs::dubins_path_finding::plan_spherical_dubins_path(request);

  EXPECT_EQ(result.status, PlannerStatus::SearchLimitReached);
  EXPECT_FALSE(result.path.has_value());
  EXPECT_EQ(result.expanded_nodes, 1U);
}

} // namespace
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
