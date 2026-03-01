// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_DUBINS_PATH_FINDING_SPHERICAL_DUBINS_PLANNER_HPP
#define CPP_HELPER_LIBS_DUBINS_PATH_FINDING_SPHERICAL_DUBINS_PLANNER_HPP

#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/quantities/length.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/small_arc.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_circle.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_polygon.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_ray.hpp"

namespace cpp_helper_libs::dubins_path_finding {

using ObstructionArea = std::variant<cpp_helper_libs::spherical_geometry::SphericalCircle,
                                     cpp_helper_libs::spherical_geometry::SphericalPolygon>;

using PathSegment = std::variant<cpp_helper_libs::spherical_geometry::MinorArc,
                                 cpp_helper_libs::spherical_geometry::SmallArc>;

/**
 * @brief Spherical environment used by the Dubins planner.
 */
struct SphericalEnvironment final {
  /// Obstacles treated as blocked areas including their boundaries.
  std::vector<ObstructionArea> obstructions;
};

/**
 * @brief Vehicle model for forward-only minimum-turn-radius motion.
 */
struct DubinsVehicleModel final {
  /// Minimum admissible turning-circle radius as a central angle on the sphere.
  cpp_helper_libs::quantities::Angle minimum_turn_radius;
};

/**
 * @brief Planner controls for spherical Dubins lattice A* search.
 */
struct PlannerConfig final {
  /// Position quantization size used to discretize latitude/longitude bins.
  cpp_helper_libs::quantities::Angle lattice_position_resolution =
      cpp_helper_libs::quantities::Angle::degrees(1.0);
  /// Number of heading bins over [0, 2*pi).
  std::size_t heading_bin_count = 72U;
  /// Forward primitive length for straight geodesic motion.
  cpp_helper_libs::quantities::Angle forward_step = cpp_helper_libs::quantities::Angle::degrees(2.0);
  /// Sweep angle for each turning primitive.
  cpp_helper_libs::quantities::Angle turn_step = cpp_helper_libs::quantities::Angle::degrees(5.0);
  /// Maximum central-angle error allowed for goal position satisfaction.
  cpp_helper_libs::quantities::Angle goal_position_tolerance =
      cpp_helper_libs::quantities::Angle::degrees(1.0);
  /// Maximum central-angle error allowed for goal heading satisfaction.
  cpp_helper_libs::quantities::Angle goal_heading_tolerance =
      cpp_helper_libs::quantities::Angle::degrees(7.5);
  /// Hard cap on expanded nodes before returning search-limit status.
  std::size_t max_expanded_nodes = 250000U;
};

/**
 * @brief Input request for spherical Dubins path planning.
 */
struct Request final {
  /// Start state (surface position + forward tangent).
  cpp_helper_libs::spherical_geometry::SphericalRay start;
  /// Goal state (surface position + desired forward tangent).
  cpp_helper_libs::spherical_geometry::SphericalRay goal;
  /// Obstacle environment.
  SphericalEnvironment environment;
  /// Minimum-turn-radius vehicle parameters.
  DubinsVehicleModel vehicle;
  /// Physical sphere radius used to convert angular length into surface length.
  cpp_helper_libs::quantities::Length sphere_radius = cpp_helper_libs::quantities::Length::meters(1.0);
  /// Search discretization and termination settings.
  PlannerConfig config{};
};

/**
 * @brief Planner termination status.
 */
enum class PlannerStatus {
  Found,
  NoPath,
  InvalidInput,
  SearchLimitReached,
};

/**
 * @brief Found path data.
 */
struct Path final {
  /// Ordered segment sequence from start to goal, using only minor and small arcs.
  std::vector<PathSegment> segments;
  /// Total angular path length on the unit sphere.
  cpp_helper_libs::quantities::Angle angular_length;
  /// Total physical surface length based on @ref Request::sphere_radius.
  cpp_helper_libs::quantities::Length surface_length;
};

/**
 * @brief Planner output.
 */
struct Result final {
  /// Termination status.
  PlannerStatus status = PlannerStatus::NoPath;
  /// Populated when status is @ref PlannerStatus::Found.
  std::optional<Path> path;
  /// Number of expanded nodes before termination.
  std::size_t expanded_nodes = 0U;
};

/**
 * @brief Plan a spherical Dubins path with obstacle avoidance.
 *
 * Operating-envelope assumption:
 * start-to-goal central-angle separation is expected to be at most 50 degrees.
 */
Result plan_spherical_dubins_path(const Request &request) noexcept;

} // namespace cpp_helper_libs::dubins_path_finding

#endif // CPP_HELPER_LIBS_DUBINS_PATH_FINDING_SPHERICAL_DUBINS_PLANNER_HPP
