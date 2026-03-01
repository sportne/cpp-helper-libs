// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#ifndef CPP_HELPER_LIBS_PATH_FINDING_A_STAR_HPP
#define CPP_HELPER_LIBS_PATH_FINDING_A_STAR_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cpp_helper_libs::path_finding {

/**
 * @brief Opaque node identifier used by the generic A* engine.
 */
using NodeId = std::uint64_t;

/**
 * @brief Directed weighted edge emitted by @ref AStarProblem::expand.
 */
struct WeightedEdge final {
  /// Destination node ID.
  NodeId to;
  /// Non-negative edge traversal cost. Invalid values are ignored by the solver.
  double cost;
  /// Domain-specific metadata carried into the chosen path output.
  std::uint64_t payload;
};

/**
 * @brief Problem interface consumed by the generic A* solver.
 */
class AStarProblem {
public:
  virtual ~AStarProblem() = default;

  /**
   * @brief Return whether @p node satisfies the goal condition.
   */
  virtual bool is_goal(NodeId node) const noexcept = 0;

  /**
   * @brief Return heuristic estimate from @p node to any goal.
   *
   * Solver ignores non-finite values.
   */
  virtual double heuristic(NodeId node) const noexcept = 0;

  /**
   * @brief Populate outgoing edges from @p node.
   *
   * Implementations should overwrite @p out_edges content each call.
   */
  virtual void expand(NodeId node, std::vector<WeightedEdge> *out_edges) = 0;
};

/**
 * @brief Runtime configuration for @ref solve_a_star.
 */
struct AStarConfig final {
  /// Hard cap on expanded nodes before returning @ref AStarStatus::SearchLimitReached.
  std::size_t max_expanded_nodes = 250000U;
};

/**
 * @brief Termination status returned by @ref solve_a_star.
 */
enum class AStarStatus {
  Found,
  NoPath,
  InvalidInput,
  SearchLimitReached,
};

/**
 * @brief Result payload for @ref solve_a_star.
 */
struct AStarResult final {
  /// Solver termination status.
  AStarStatus status = AStarStatus::NoPath;
  /// Chosen node sequence from start to goal (inclusive) when status is Found.
  std::vector<NodeId> node_path;
  /// Payload sequence for selected edges. Size is node_path.size() - 1 when Found.
  std::vector<std::uint64_t> edge_payload_path;
  /// Total accumulated edge cost for the selected path.
  double total_cost = 0.0;
  /// Number of expanded nodes before termination.
  std::size_t expanded_nodes = 0U;
};

/**
 * @brief Solve a shortest-path problem using A* search.
 *
 * The solver assumes non-negative edge costs and a finite heuristic.
 * Invalid costs/heuristics are ignored defensively.
 */
AStarResult solve_a_star(AStarProblem &problem, NodeId start,
                         const AStarConfig &config = {}) noexcept;

} // namespace cpp_helper_libs::path_finding

#endif // CPP_HELPER_LIBS_PATH_FINDING_A_STAR_HPP
