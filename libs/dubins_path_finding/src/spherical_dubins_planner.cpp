// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/dubins_path_finding/spherical_dubins_planner.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "cpp_helper_libs/linear_algebra/unit_vector3.hpp"
#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/path_finding/a_star.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_curve.hpp"

namespace cpp_helper_libs::dubins_path_finding {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
constexpr double kDegreesToRadians = std::numbers::pi_v<double> / 180.0;
// Expected operating envelope: requests should remain local (<= 50 degrees).
constexpr double kMaxAssumedPathSeparationRadians = 50.0 * kDegreesToRadians;
constexpr std::size_t kHashMixConstant = 0x9e3779b9U;
constexpr unsigned int kHashShiftLeftBits = 6U;
constexpr unsigned int kHashShiftRightBits = 2U;

void hash_combine(std::size_t *seed, const std::size_t value) noexcept {
  *seed ^=
      value + kHashMixConstant + (*seed << kHashShiftLeftBits) + (*seed >> kHashShiftRightBits);
}

struct NodeKey final {
  int latitude_bin;
  int longitude_bin;
  int heading_bin;

  bool operator==(const NodeKey &other) const noexcept {
    return latitude_bin == other.latitude_bin && longitude_bin == other.longitude_bin &&
           heading_bin == other.heading_bin;
  }
};

struct NodeKeyHash final {
  std::size_t operator()(const NodeKey &key) const noexcept {
    std::size_t seed = 0U;
    hash_combine(&seed, static_cast<std::size_t>(key.latitude_bin));
    hash_combine(&seed, static_cast<std::size_t>(key.longitude_bin));
    hash_combine(&seed, static_cast<std::size_t>(key.heading_bin));
    return seed;
  }
};

struct NodeState final {
  std::optional<cpp_helper_libs::spherical_geometry::SphericalRay> ray;
};

bool is_finite_and_positive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

bool is_finite_and_non_negative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

bool validate_request(const Request &request) noexcept {
  const double position_resolution_radians = request.config.lattice_position_resolution.in(
      cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double forward_step_radians =
      request.config.forward_step.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double turn_step_radians =
      request.config.turn_step.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double minimum_turn_radius_radians =
      request.vehicle.minimum_turn_radius.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double goal_position_tolerance_radians =
      request.config.goal_position_tolerance.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double goal_heading_tolerance_radians =
      request.config.goal_heading_tolerance.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
  const double sphere_radius_meters =
      request.sphere_radius.in(cpp_helper_libs::quantities::Length::Unit::Meter);

  if (!is_finite_and_positive(position_resolution_radians) || position_resolution_radians >= kPi) {
    return false;
  }

  if (request.config.heading_bin_count == 0U) {
    return false;
  }

  if (!is_finite_and_positive(forward_step_radians) || forward_step_radians >= kPi) {
    return false;
  }

  if (!is_finite_and_positive(turn_step_radians) || turn_step_radians >= kTwoPi) {
    return false;
  }

  if (!is_finite_and_positive(minimum_turn_radius_radians) || minimum_turn_radius_radians >= kPi) {
    return false;
  }

  if (!is_finite_and_non_negative(goal_position_tolerance_radians) ||
      goal_position_tolerance_radians > kPi) {
    return false;
  }

  if (!is_finite_and_non_negative(goal_heading_tolerance_radians) ||
      goal_heading_tolerance_radians > kPi) {
    return false;
  }

  if (!is_finite_and_positive(sphere_radius_meters)) {
    return false;
  }

  if (request.config.max_expanded_nodes == 0U) {
    return false;
  }

  const double start_goal_separation =
      request.start.radial().central_angle_radians(request.goal.radial());
  return std::isfinite(start_goal_separation) &&
         start_goal_separation <= kMaxAssumedPathSeparationRadians;
}

bool any_obstacle_contains(const std::vector<ObstructionArea> &obstructions,
                           const cpp_helper_libs::linear_algebra::UnitVector3 &point) {
  for (const ObstructionArea &obstruction : obstructions) {
    const bool contains = std::visit(
        [&](const auto &shape) { return shape.contains_inclusive_exact(point); }, obstruction);
    if (contains) {
      return true;
    }
  }

  return false;
}

bool any_obstacle_boundary_intersects(
    const std::vector<ObstructionArea> &obstructions,
    const cpp_helper_libs::spherical_geometry::SphericalCurve &curve) {
  for (const ObstructionArea &obstruction : obstructions) {
    const bool intersects = std::visit(
        [&](const auto &shape) { return shape.boundary_intersects_inclusive_exact(curve); },
        obstruction);
    if (intersects) {
      return true;
    }
  }

  return false;
}

class SphericalDubinsProblem final : public cpp_helper_libs::path_finding::AStarProblem {
public:
  explicit SphericalDubinsProblem(Request request)
      : request_(std::move(request)), longitude_bin_count_(compute_longitude_bin_count(request_)),
        heading_bin_width_radians_(kTwoPi / static_cast<double>(request_.config.heading_bin_count)),
        start_node_id_(ensure_node_for_ray(request_.start)) {}

  cpp_helper_libs::path_finding::NodeId start_node_id() const noexcept { return start_node_id_; }

  bool is_goal(const cpp_helper_libs::path_finding::NodeId node) const noexcept override {
    try {
      const cpp_helper_libs::spherical_geometry::SphericalRay *ray = ray_for_node(node);
      if (ray == nullptr) {
        return false;
      }

      const double position_error = ray->radial().central_angle_radians(request_.goal.radial());
      if (position_error > request_.config.goal_position_tolerance.in(
                               cpp_helper_libs::quantities::Angle::Unit::Radian)) {
        return false;
      }

      const double heading_error = ray->tangent().central_angle_radians(request_.goal.tangent());
      return heading_error <= request_.config.goal_heading_tolerance.in(
                                  cpp_helper_libs::quantities::Angle::Unit::Radian);
    } catch (...) {
      return false;
    }
  }

  double heuristic(const cpp_helper_libs::path_finding::NodeId node) const noexcept override {
    try {
      const cpp_helper_libs::spherical_geometry::SphericalRay *ray = ray_for_node(node);
      if (ray == nullptr) {
        return kPi;
      }
      return ray->radial().central_angle_radians(request_.goal.radial());
    } catch (...) {
      return kPi;
    }
  }

  void expand(const cpp_helper_libs::path_finding::NodeId node,
              std::vector<cpp_helper_libs::path_finding::WeightedEdge> *out_edges) override {
    out_edges->clear();
    const cpp_helper_libs::spherical_geometry::SphericalRay *ray = ray_for_node(node);
    if (ray == nullptr) {
      return;
    }
    const cpp_helper_libs::spherical_geometry::SphericalRay current_ray = *ray;

    const std::optional<cpp_helper_libs::spherical_geometry::MinorArc> straight_segment =
        cpp_helper_libs::spherical_geometry::MinorArc::from_start_and_sweep(
            current_ray, request_.config.forward_step);
    if (straight_segment.has_value()) {
      maybe_add_segment(*straight_segment, out_edges);
    }

    const std::optional<cpp_helper_libs::spherical_geometry::SmallArc> left_segment =
        build_turn_segment(current_ray,
                           cpp_helper_libs::spherical_geometry::TurnDirection::CounterClockwise);
    if (left_segment.has_value()) {
      maybe_add_segment(*left_segment, out_edges);
    }

    const std::optional<cpp_helper_libs::spherical_geometry::SmallArc> right_segment =
        build_turn_segment(current_ray,
                           cpp_helper_libs::spherical_geometry::TurnDirection::Clockwise);
    if (right_segment.has_value()) {
      maybe_add_segment(*right_segment, out_edges);
    }
  }

  std::optional<Path> build_path_from_search_result(
      const cpp_helper_libs::path_finding::AStarResult &search_result) const {
    if (search_result.status != cpp_helper_libs::path_finding::AStarStatus::Found) {
      return std::nullopt;
    }

    std::vector<PathSegment> segments;
    segments.reserve(search_result.edge_payload_path.size());

    double total_length_radians = 0.0;

    for (const std::uint64_t payload : search_result.edge_payload_path) {
      if (payload >= payload_segments_.size()) {
        return std::nullopt;
      }

      const PathSegment &segment = payload_segments_[payload];
      total_length_radians += std::visit(
          [](const auto &value) {
            return value.length().in(cpp_helper_libs::quantities::Angle::Unit::Radian);
          },
          segment);
      segments.push_back(segment);
    }

    const cpp_helper_libs::quantities::Angle angular_length =
        cpp_helper_libs::quantities::Angle::radians(total_length_radians);
    const cpp_helper_libs::quantities::Length surface_length =
        request_.sphere_radius.multiplied_by(total_length_radians);

    return Path{.segments = std::move(segments),
                .angular_length = angular_length,
                .surface_length = surface_length};
  }

private:
  bool is_known_node(const cpp_helper_libs::path_finding::NodeId node) const noexcept {
    return static_cast<std::size_t>(node) < states_.size();
  }

  const cpp_helper_libs::spherical_geometry::SphericalRay *
  ray_for_node(const cpp_helper_libs::path_finding::NodeId node) const noexcept {
    if (!is_known_node(node)) {
      return nullptr;
    }

    const std::optional<cpp_helper_libs::spherical_geometry::SphericalRay> &ray =
        states_[static_cast<std::size_t>(node)].ray;
    if (!ray.has_value()) {
      return nullptr;
    }

    return &(*ray);
  }

  static int compute_longitude_bin_count(const Request &request) {
    const double resolution_degrees = request.config.lattice_position_resolution.in(
        cpp_helper_libs::quantities::Angle::Unit::Degree);
    const int bins = static_cast<int>(std::lround(360.0 / resolution_degrees));
    return bins > 0 ? bins : 1;
  }

  static int wrap_longitude_bin(const int bin, const int count) noexcept {
    const int modulo = bin % count;
    return modulo < 0 ? modulo + count : modulo;
  }

  int heading_bin_for_ray(const cpp_helper_libs::spherical_geometry::SphericalRay &ray) const {
    const cpp_helper_libs::spherical_geometry::Coordinate coordinate =
        cpp_helper_libs::spherical_geometry::Coordinate::from_radial(ray.radial());
    const double longitude_radians =
        coordinate.longitude().in(cpp_helper_libs::quantities::Angle::Unit::Radian);

    const cpp_helper_libs::linear_algebra::Vector3 east_guess(-std::sin(longitude_radians),
                                                              std::cos(longitude_radians), 0.0);
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> east =
        cpp_helper_libs::linear_algebra::UnitVector3::from_vector(east_guess);
    if (!east.has_value()) {
      return 0;
    }
    const cpp_helper_libs::linear_algebra::UnitVector3 east_vector = *east;

    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> north =
        cpp_helper_libs::linear_algebra::UnitVector3::from_vector(ray.radial().cross(east_vector));
    if (!north.has_value()) {
      return 0;
    }
    const cpp_helper_libs::linear_algebra::UnitVector3 north_vector = *north;

    double heading = std::atan2(ray.tangent().dot(east_vector), ray.tangent().dot(north_vector));
    if (heading < 0.0) {
      heading += kTwoPi;
    }

    const int bin = static_cast<int>(std::floor(heading / heading_bin_width_radians_));
    const int bin_count = static_cast<int>(request_.config.heading_bin_count);
    return bin == bin_count ? 0 : bin;
  }

  NodeKey discretize_ray(const cpp_helper_libs::spherical_geometry::SphericalRay &ray) const {
    const cpp_helper_libs::spherical_geometry::Coordinate coordinate =
        cpp_helper_libs::spherical_geometry::Coordinate::from_radial(ray.radial());

    const double resolution_degrees = request_.config.lattice_position_resolution.in(
        cpp_helper_libs::quantities::Angle::Unit::Degree);
    const double latitude_degrees =
        coordinate.latitude().in(cpp_helper_libs::quantities::Angle::Unit::Degree);
    const double longitude_degrees =
        coordinate.longitude().in(cpp_helper_libs::quantities::Angle::Unit::Degree);

    const int latitude_bin =
        static_cast<int>(std::lround((latitude_degrees + 90.0) / resolution_degrees));
    const int longitude_bin = wrap_longitude_bin(
        static_cast<int>(std::lround((longitude_degrees + 180.0) / resolution_degrees)),
        longitude_bin_count_);
    const int heading_bin = heading_bin_for_ray(ray);

    return NodeKey{
        .latitude_bin = latitude_bin, .longitude_bin = longitude_bin, .heading_bin = heading_bin};
  }

  cpp_helper_libs::path_finding::NodeId
  ensure_node_for_ray(const cpp_helper_libs::spherical_geometry::SphericalRay &ray) {
    const NodeKey key = discretize_ray(ray);

    const auto existing_node_iterator = node_ids_by_key_.find(key);
    if (existing_node_iterator != node_ids_by_key_.end()) {
      states_[static_cast<std::size_t>(existing_node_iterator->second)].ray.emplace(ray);
      return existing_node_iterator->second;
    }

    const auto node_id = static_cast<cpp_helper_libs::path_finding::NodeId>(states_.size());
    states_.push_back(NodeState{.ray = ray});
    node_ids_by_key_.emplace(key, node_id);
    return node_id;
  }

  bool segment_is_collision_free(
      const cpp_helper_libs::spherical_geometry::SphericalCurve &segment) const {
    const cpp_helper_libs::linear_algebra::UnitVector3 start = segment.start_radial();
    const cpp_helper_libs::linear_algebra::UnitVector3 end = segment.end_radial();
    const cpp_helper_libs::linear_algebra::UnitVector3 midpoint = segment.point_at_parameter(0.5);

    if (any_obstacle_contains(request_.environment.obstructions, start) ||
        any_obstacle_contains(request_.environment.obstructions, end) ||
        any_obstacle_contains(request_.environment.obstructions, midpoint)) {
      return false;
    }

    if (any_obstacle_boundary_intersects(request_.environment.obstructions, segment)) {
      return false;
    }

    return true;
  }

  std::optional<cpp_helper_libs::spherical_geometry::SmallArc>
  build_turn_segment(const cpp_helper_libs::spherical_geometry::SphericalRay &start_ray,
                     const cpp_helper_libs::spherical_geometry::TurnDirection direction) const {
    const double turn_radius_radians =
        request_.vehicle.minimum_turn_radius.in(cpp_helper_libs::quantities::Angle::Unit::Radian);
    const double sign =
        direction == cpp_helper_libs::spherical_geometry::TurnDirection::CounterClockwise ? 1.0
                                                                                          : -1.0;

    const cpp_helper_libs::linear_algebra::Vector3 center_vector =
        start_ray.radial().scaled_by(std::cos(turn_radius_radians)) +
        start_ray.normal().scaled_by(sign * std::sin(turn_radius_radians));
    const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> center =
        cpp_helper_libs::linear_algebra::UnitVector3::from_vector(center_vector);
    if (!center.has_value()) {
      return std::nullopt;
    }

    return cpp_helper_libs::spherical_geometry::SmallArc::from_center_start_direction_and_sweep(
        *center, request_.vehicle.minimum_turn_radius, start_ray, direction,
        request_.config.turn_step);
  }

  template <typename SegmentType>
  void maybe_add_segment(const SegmentType &segment,
                         std::vector<cpp_helper_libs::path_finding::WeightedEdge> *out_edges) {
    const cpp_helper_libs::spherical_geometry::SphericalCurve &curve = segment;
    if (!segment_is_collision_free(curve)) {
      return;
    }

    const cpp_helper_libs::path_finding::NodeId destination =
        ensure_node_for_ray(segment.end_ray());
    const auto payload_id = static_cast<std::uint64_t>(payload_segments_.size());
    payload_segments_.push_back(PathSegment(segment));

    out_edges->push_back(cpp_helper_libs::path_finding::WeightedEdge{
        .to = destination,
        .cost = segment.length().in(cpp_helper_libs::quantities::Angle::Unit::Radian),
        .payload = payload_id,
    });
  }

  Request request_;
  int longitude_bin_count_;
  double heading_bin_width_radians_;
  std::unordered_map<NodeKey, cpp_helper_libs::path_finding::NodeId, NodeKeyHash> node_ids_by_key_;
  std::vector<NodeState> states_;
  std::vector<PathSegment> payload_segments_;
  cpp_helper_libs::path_finding::NodeId start_node_id_;
};

PlannerStatus map_status(const cpp_helper_libs::path_finding::AStarStatus status) {
  switch (status) {
  case cpp_helper_libs::path_finding::AStarStatus::Found:
    return PlannerStatus::Found;
  case cpp_helper_libs::path_finding::AStarStatus::NoPath:
    return PlannerStatus::NoPath;
  case cpp_helper_libs::path_finding::AStarStatus::InvalidInput:
    return PlannerStatus::InvalidInput;
  case cpp_helper_libs::path_finding::AStarStatus::SearchLimitReached:
    return PlannerStatus::SearchLimitReached;
  }

  return PlannerStatus::NoPath;
}

} // namespace

Result plan_spherical_dubins_path(const Request &request) noexcept {
  try {
    Result result{};

    if (!validate_request(request)) {
      result.status = PlannerStatus::InvalidInput;
      return result;
    }

    if (any_obstacle_contains(request.environment.obstructions, request.start.radial()) ||
        any_obstacle_contains(request.environment.obstructions, request.goal.radial())) {
      result.status = PlannerStatus::InvalidInput;
      return result;
    }

    SphericalDubinsProblem problem(request);

    cpp_helper_libs::path_finding::AStarConfig search_config{};
    search_config.max_expanded_nodes = request.config.max_expanded_nodes;

    const cpp_helper_libs::path_finding::AStarResult search_result =
        cpp_helper_libs::path_finding::solve_a_star(problem, problem.start_node_id(),
                                                    search_config);

    result.status = map_status(search_result.status);
    result.expanded_nodes = search_result.expanded_nodes;

    if (search_result.status != cpp_helper_libs::path_finding::AStarStatus::Found) {
      return result;
    }

    const std::optional<Path> path = problem.build_path_from_search_result(search_result);
    if (!path.has_value()) {
      result.status = PlannerStatus::NoPath;
      return result;
    }

    result.path.emplace(*path);
    return result;
  } catch (...) {
    Result result{};
    result.status = PlannerStatus::InvalidInput;
    return result;
  }
}

} // namespace cpp_helper_libs::dubins_path_finding
