#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "cpp_helper_libs/path_finding/a_star.hpp"

namespace {
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

using cpp_helper_libs::path_finding::AStarConfig;
using cpp_helper_libs::path_finding::AStarProblem;
using cpp_helper_libs::path_finding::AStarResult;
using cpp_helper_libs::path_finding::AStarStatus;
using cpp_helper_libs::path_finding::NodeId;
using cpp_helper_libs::path_finding::WeightedEdge;

class ToyGraphProblem final : public AStarProblem {
public:
  ToyGraphProblem(std::unordered_map<NodeId, std::vector<WeightedEdge>> adjacency,
                  std::unordered_map<NodeId, double> heuristics,
                  std::unordered_set<NodeId> goal_nodes) noexcept
      : adjacency_(std::move(adjacency)), heuristics_(std::move(heuristics)),
        goal_nodes_(std::move(goal_nodes)) {}

  bool is_goal(const NodeId node) const noexcept override { return goal_nodes_.contains(node); }

  double heuristic(const NodeId node) const noexcept override {
    const auto heuristic_iterator = heuristics_.find(node);
    if (heuristic_iterator == heuristics_.end()) {
      return std::numeric_limits<double>::infinity();
    }

    return heuristic_iterator->second;
  }

  void expand(const NodeId node, std::vector<WeightedEdge> *out_edges) override {
    out_edges->clear();

    const auto adjacency_iterator = adjacency_.find(node);
    if (adjacency_iterator == adjacency_.end()) {
      return;
    }

    *out_edges = adjacency_iterator->second;
  }

private:
  std::unordered_map<NodeId, std::vector<WeightedEdge>> adjacency_;
  std::unordered_map<NodeId, double> heuristics_;
  std::unordered_set<NodeId> goal_nodes_;
};

TEST(AStarTest, FindsShortestWeightedPathAndPayloadSequence) {
  ToyGraphProblem problem({{1U, {{2U, 1.0, 12U}, {3U, 4.0, 13U}}},
                           {2U, {{3U, 1.0, 23U}, {4U, 5.0, 24U}}},
                           {3U, {{4U, 1.0, 34U}, {5U, 10.0, 35U}}},
                           {4U, {{5U, 1.0, 45U}}}},
                          {{1U, 3.0}, {2U, 2.0}, {3U, 1.0}, {4U, 1.0}, {5U, 0.0}}, {5U});

  const AStarResult result = cpp_helper_libs::path_finding::solve_a_star(problem, 1U);

  EXPECT_EQ(result.status, AStarStatus::Found);
  EXPECT_EQ(result.node_path, (std::vector<NodeId>{1U, 2U, 3U, 4U, 5U}));
  EXPECT_EQ(result.edge_payload_path, (std::vector<std::uint64_t>{12U, 23U, 34U, 45U}));
  EXPECT_DOUBLE_EQ(result.total_cost, 4.0);
  EXPECT_GT(result.expanded_nodes, 0U);
}

TEST(AStarTest, ReturnsNoPathWhenGoalCannotBeReached) {
  ToyGraphProblem problem({{1U, {{2U, 1.0, 12U}}}, {2U, {{3U, 1.0, 23U}}}},
                          {{1U, 2.0}, {2U, 1.0}, {3U, 1.0}, {9U, 0.0}}, {9U});

  const AStarResult result = cpp_helper_libs::path_finding::solve_a_star(problem, 1U);

  EXPECT_EQ(result.status, AStarStatus::NoPath);
  EXPECT_TRUE(result.node_path.empty());
  EXPECT_TRUE(result.edge_payload_path.empty());
}

TEST(AStarTest, HonorsExpansionLimit) {
  ToyGraphProblem problem({{1U, {{2U, 1.0, 12U}}}, {2U, {{3U, 1.0, 23U}}}, {3U, {{4U, 1.0, 34U}}}},
                          {{1U, 3.0}, {2U, 2.0}, {3U, 1.0}, {4U, 0.0}}, {4U});

  AStarConfig config{};
  config.max_expanded_nodes = 1U;

  const AStarResult result = cpp_helper_libs::path_finding::solve_a_star(problem, 1U, config);

  EXPECT_EQ(result.status, AStarStatus::SearchLimitReached);
  EXPECT_EQ(result.expanded_nodes, 1U);
}

TEST(AStarTest, IgnoresInvalidEdgesAndStillFindsPath) {
  ToyGraphProblem problem({{1U,
                            {{2U, -1.0, 12U},
                             {3U, std::numeric_limits<double>::quiet_NaN(), 13U},
                             {4U, std::numeric_limits<double>::infinity(), 14U},
                             {5U, 2.0, 15U}}}},
                          {{1U, 1.0}, {5U, 0.0}}, {5U});

  const AStarResult result = cpp_helper_libs::path_finding::solve_a_star(problem, 1U);

  EXPECT_EQ(result.status, AStarStatus::Found);
  EXPECT_EQ(result.node_path, (std::vector<NodeId>{1U, 5U}));
  EXPECT_EQ(result.edge_payload_path, (std::vector<std::uint64_t>{15U}));
  EXPECT_DOUBLE_EQ(result.total_cost, 2.0);
}

TEST(AStarTest, RejectsInvalidConfig) {
  ToyGraphProblem problem({}, {{1U, 0.0}}, {1U});

  AStarConfig config{};
  config.max_expanded_nodes = 0U;

  const AStarResult result = cpp_helper_libs::path_finding::solve_a_star(problem, 1U, config);

  EXPECT_EQ(result.status, AStarStatus::InvalidInput);
  EXPECT_EQ(result.expanded_nodes, 0U);
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
} // namespace
