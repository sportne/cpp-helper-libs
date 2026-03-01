// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include "cpp_helper_libs/path_finding/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cpp_helper_libs::path_finding {
namespace {

struct OpenEntry final {
  NodeId node;
  double g_cost;
  double f_cost;
  std::uint64_t insertion_order;
};

struct OpenEntryGreater final {
  bool operator()(const OpenEntry &left, const OpenEntry &right) const noexcept {
    if (left.f_cost != right.f_cost) {
      return left.f_cost > right.f_cost;
    }

    return left.insertion_order > right.insertion_order;
  }
};

struct ParentRecord final {
  NodeId parent;
  std::uint64_t payload;
};

struct SolverState final {
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open_set;
  std::unordered_map<NodeId, double> best_g_cost;
  std::unordered_map<NodeId, ParentRecord> parents;
  std::uint64_t insertion_order = 1U;
};

std::optional<std::vector<NodeId>> reconstruct_node_path(
    const std::unordered_map<NodeId, ParentRecord> &parents, const NodeId goal) {
  std::vector<NodeId> reversed_path;
  reversed_path.push_back(goal);

  NodeId current = goal;
  while (true) {
    const auto parent_iterator = parents.find(current);
    if (parent_iterator == parents.end()) {
      break;
    }
    current = parent_iterator->second.parent;
    reversed_path.push_back(current);
  }

  std::reverse(reversed_path.begin(), reversed_path.end());
  return reversed_path;
}

std::optional<std::vector<std::uint64_t>> reconstruct_payload_path(
    const std::unordered_map<NodeId, ParentRecord> &parents, const std::vector<NodeId> &node_path) {
  if (node_path.empty()) {
    return std::vector<std::uint64_t>{};
  }

  std::vector<std::uint64_t> payload_path;
  payload_path.reserve(node_path.size() - 1U);

  for (std::size_t index = 1U; index < node_path.size(); ++index) {
    const auto parent_iterator = parents.find(node_path[index]);
    if (parent_iterator == parents.end()) {
      return std::nullopt;
    }

    payload_path.push_back(parent_iterator->second.payload);
  }

  return payload_path;
}

bool is_edge_cost_valid(const double edge_cost) noexcept {
  return std::isfinite(edge_cost) && edge_cost >= 0.0;
}

bool is_heuristic_valid(const double heuristic_value) noexcept { return std::isfinite(heuristic_value); }

bool initialize_search(const AStarProblem &problem, const NodeId start, SolverState *state) {
  const double start_heuristic = problem.heuristic(start);
  if (!is_heuristic_valid(start_heuristic)) {
    return false;
  }

  state->best_g_cost[start] = 0.0;
  state->open_set.push(OpenEntry{.node = start,
                                 .g_cost = 0.0,
                                 .f_cost = start_heuristic,
                                 .insertion_order = 0U});
  return true;
}

std::optional<OpenEntry> pop_next_valid_entry(SolverState *state) {
  while (!state->open_set.empty()) {
    const OpenEntry current = state->open_set.top();
    state->open_set.pop();

    const auto best_cost_iterator = state->best_g_cost.find(current.node);
    if (best_cost_iterator == state->best_g_cost.end()) {
      continue;
    }

    if (current.g_cost != best_cost_iterator->second) {
      continue;
    }

    return current;
  }

  return std::nullopt;
}

std::optional<AStarResult>
build_found_result(const std::unordered_map<NodeId, ParentRecord> &parents, const OpenEntry &goal_entry,
                   const std::size_t expanded_nodes) {
  const std::optional<std::vector<NodeId>> node_path = reconstruct_node_path(parents, goal_entry.node);
  if (!node_path.has_value()) {
    return std::nullopt;
  }

  const std::optional<std::vector<std::uint64_t>> payload_path =
      reconstruct_payload_path(parents, *node_path);
  if (!payload_path.has_value()) {
    return std::nullopt;
  }

  AStarResult result{};
  result.status = AStarStatus::Found;
  result.node_path = *node_path;
  result.edge_payload_path = *payload_path;
  result.total_cost = goal_entry.g_cost;
  result.expanded_nodes = expanded_nodes;
  return result;
}

void expand_neighbors(AStarProblem &problem, const OpenEntry &current, SolverState *state) {
  std::vector<WeightedEdge> edges;
  problem.expand(current.node, &edges);

  for (const WeightedEdge &edge : edges) {
    if (!is_edge_cost_valid(edge.cost)) {
      continue;
    }

    const double candidate_g_cost = current.g_cost + edge.cost;
    if (!std::isfinite(candidate_g_cost)) {
      continue;
    }

    const auto neighbor_best_iterator = state->best_g_cost.find(edge.to);
    if (neighbor_best_iterator != state->best_g_cost.end() &&
        candidate_g_cost >= neighbor_best_iterator->second) {
      continue;
    }

    const double neighbor_heuristic = problem.heuristic(edge.to);
    if (!is_heuristic_valid(neighbor_heuristic)) {
      continue;
    }

    const double candidate_f_cost = candidate_g_cost + neighbor_heuristic;
    if (!std::isfinite(candidate_f_cost)) {
      continue;
    }

    state->best_g_cost[edge.to] = candidate_g_cost;
    state->parents[edge.to] = ParentRecord{.parent = current.node, .payload = edge.payload};
    state->open_set.push(OpenEntry{.node = edge.to,
                                   .g_cost = candidate_g_cost,
                                   .f_cost = candidate_f_cost,
                                   .insertion_order = state->insertion_order});
    ++state->insertion_order;
  }
}

} // namespace

AStarResult solve_a_star(AStarProblem &problem, const NodeId start,
                         const AStarConfig &config) noexcept {
  AStarResult result{};

  if (config.max_expanded_nodes == 0U) {
    result.status = AStarStatus::InvalidInput;
    return result;
  }

  SolverState state;
  if (!initialize_search(problem, start, &state)) {
    result.status = AStarStatus::InvalidInput;
    return result;
  }

  while (true) {
    const std::optional<OpenEntry> current = pop_next_valid_entry(&state);
    if (!current.has_value()) {
      break;
    }

    if (result.expanded_nodes >= config.max_expanded_nodes) {
      result.status = AStarStatus::SearchLimitReached;
      return result;
    }

    ++result.expanded_nodes;

    if (problem.is_goal(current->node)) {
      const std::optional<AStarResult> found_result =
          build_found_result(state.parents, *current, result.expanded_nodes);
      if (!found_result.has_value()) {
        result.status = AStarStatus::NoPath;
        return result;
      }

      return *found_result;
    }

    expand_neighbors(problem, *current, &state);
  }

  result.status = AStarStatus::NoPath;
  return result;
}

} // namespace cpp_helper_libs::path_finding
