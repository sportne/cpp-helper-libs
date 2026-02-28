// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cpp_helper_libs/linear_algebra/vector3.hpp"
#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"

namespace {

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::spherical_geometry::Coordinate;
using cpp_helper_libs::spherical_geometry::MinorArc;

// Size sweep used by all benchmarks: small, medium, and large datasets.
constexpr std::size_t kSmallFixtureSize = 128U;
constexpr std::size_t kMediumFixtureSize = 512U;
constexpr std::size_t kLargeFixtureSize = 2048U;
constexpr std::size_t kMaxFixtureBuildAttemptsMultiplier = 20U;

template <typename ValueType, typename Builder>
const std::vector<ValueType> &
fixture_for_size(std::unordered_map<std::size_t, std::vector<ValueType>> *cache,
                 const std::size_t size, const Builder &builder) {
  const auto existing = cache->find(size);
  if (existing != cache->end()) {
    return existing->second;
  }

  const auto inserted = cache->emplace(size, builder(size));
  return inserted.first->second;
}

std::vector<Coordinate> build_coordinate_fixture(const std::size_t count) {
  std::vector<Coordinate> coordinates;
  coordinates.reserve(count);

  for (std::size_t index = 0U; index < count; ++index) {
    const double latitude = -80.0 + static_cast<double>(index % 161U);
    const double longitude = -170.0 + static_cast<double>((index * 37U) % 341U);
    coordinates.push_back(Coordinate::degrees(latitude, longitude));
  }

  return coordinates;
}

std::vector<UnitVector3> build_radial_fixture(const std::size_t count) {
  const std::vector<Coordinate> coordinates = build_coordinate_fixture(count);
  std::vector<UnitVector3> radials;
  radials.reserve(coordinates.size());

  for (const Coordinate &coordinate : coordinates) {
    radials.push_back(coordinate.to_radial());
  }

  return radials;
}

std::vector<std::pair<UnitVector3, UnitVector3>>
build_minor_arc_endpoint_fixture(const std::size_t count) {
  std::vector<std::pair<UnitVector3, UnitVector3>> endpoints;
  endpoints.reserve(count);

  for (std::size_t index = 0U; index < count; ++index) {
    const double base_longitude = -150.0 + static_cast<double>((index * 17U) % 301U);
    const UnitVector3 start = Coordinate::degrees(-20.0, base_longitude).to_radial();
    const UnitVector3 end = Coordinate::degrees(25.0, base_longitude + 35.0).to_radial();
    endpoints.emplace_back(start, end);
  }

  return endpoints;
}

struct ArcPair final {
  MinorArc first;
  MinorArc second;
};

std::vector<ArcPair> build_minor_arc_intersection_fixture(const std::size_t count) {
  std::vector<ArcPair> pairs;
  pairs.reserve(count);

  const std::size_t max_attempts = (count * kMaxFixtureBuildAttemptsMultiplier) + 128U;
  for (std::size_t attempt = 0U; attempt < max_attempts && pairs.size() < count; ++attempt) {
    const double base_longitude = -160.0 + static_cast<double>((attempt * 13U) % 321U);

    const UnitVector3 first_start = Coordinate::degrees(-30.0, base_longitude).to_radial();
    const UnitVector3 first_end = Coordinate::degrees(30.0, base_longitude + 70.0).to_radial();
    const UnitVector3 second_start = Coordinate::degrees(-30.0, base_longitude + 35.0).to_radial();
    const UnitVector3 second_end = Coordinate::degrees(30.0, base_longitude - 35.0).to_radial();

    const std::optional<MinorArc> first_arc = MinorArc::from_endpoints(first_start, first_end);
    const std::optional<MinorArc> second_arc = MinorArc::from_endpoints(second_start, second_end);
    if (!first_arc.has_value() || !second_arc.has_value()) {
      continue;
    }

    pairs.push_back(ArcPair{.first = first_arc.value(), .second = second_arc.value()});
  }

  return pairs;
}

void BM_CoordinateToRadial(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<Coordinate>> coordinate_cache;
  const std::vector<Coordinate> &coordinates =
      fixture_for_size(&coordinate_cache, fixture_size, build_coordinate_fixture);
  double checksum = 0.0;

  for (auto _ : state) {
    for (const Coordinate &coordinate : coordinates) {
      const UnitVector3 radial = coordinate.to_radial();
      checksum += radial.x();
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(coordinates.size()));
}

void BM_CoordinateFromRadial(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<UnitVector3>> radial_cache;
  const std::vector<UnitVector3> &radials =
      fixture_for_size(&radial_cache, fixture_size, build_radial_fixture);
  double checksum = 0.0;

  for (auto _ : state) {
    for (const UnitVector3 &radial : radials) {
      const Coordinate coordinate = Coordinate::from_radial(radial);
      checksum += coordinate.latitude().in(cpp_helper_libs::quantities::Angle::Unit::Radian);
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(radials.size()));
}

void BM_MinorArcFromEndpoints(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<std::pair<UnitVector3, UnitVector3>>>
      endpoint_cache;
  const std::vector<std::pair<UnitVector3, UnitVector3>> &endpoints =
      fixture_for_size(&endpoint_cache, fixture_size, build_minor_arc_endpoint_fixture);
  std::size_t arcs_built = 0U;

  for (auto _ : state) {
    for (const auto &endpoint_pair : endpoints) {
      std::optional<MinorArc> arc =
          MinorArc::from_endpoints(endpoint_pair.first, endpoint_pair.second);
      if (!arc.has_value()) {
        state.SkipWithError("MinorArc::from_endpoints unexpectedly returned nullopt");
        return;
      }
      ++arcs_built;
      MinorArc &resolved_arc = arc.value();
      benchmark::DoNotOptimize(&resolved_arc);
    }
  }

  benchmark::DoNotOptimize(arcs_built);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(endpoints.size()));
}

void BM_MinorArcCentralAngle(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<std::pair<UnitVector3, UnitVector3>>>
      endpoint_cache;
  const std::vector<std::pair<UnitVector3, UnitVector3>> &endpoints =
      fixture_for_size(&endpoint_cache, fixture_size, build_minor_arc_endpoint_fixture);
  double checksum = 0.0;

  for (auto _ : state) {
    for (const auto &endpoint_pair : endpoints) {
      checksum += endpoint_pair.first.central_angle_radians(endpoint_pair.second);
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(endpoints.size()));
}

void BM_MinorArcUnitNormal(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<std::pair<UnitVector3, UnitVector3>>>
      endpoint_cache;
  const std::vector<std::pair<UnitVector3, UnitVector3>> &endpoints =
      fixture_for_size(&endpoint_cache, fixture_size, build_minor_arc_endpoint_fixture);
  double checksum = 0.0;

  for (auto _ : state) {
    for (const auto &endpoint_pair : endpoints) {
      const std::optional<cpp_helper_libs::linear_algebra::UnitVector3> normal =
          cpp_helper_libs::linear_algebra::unit_normal(endpoint_pair.first.as_vector(),
                                                       endpoint_pair.second.as_vector());
      if (!normal.has_value()) {
        state.SkipWithError("unit_normal returned nullopt for benchmark fixture");
        return;
      }

      checksum += normal->x();
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(endpoints.size()));
}

void BM_MinorArcIntersections(benchmark::State &state) {
  const std::size_t fixture_size = static_cast<std::size_t>(state.range(0));
  static std::unordered_map<std::size_t, std::vector<ArcPair>> arc_pair_cache;
  const std::vector<ArcPair> &arc_pairs =
      fixture_for_size(&arc_pair_cache, fixture_size, build_minor_arc_intersection_fixture);
  if (arc_pairs.empty()) {
    state.SkipWithError("No valid MinorArc fixtures available for intersections benchmark");
    return;
  }

  std::size_t intersection_count = 0U;

  for (auto _ : state) {
    for (const ArcPair &pair : arc_pairs) {
      std::vector<cpp_helper_libs::spherical_geometry::CurveIntersection> intersections =
          pair.first.intersections_with(pair.second);
      intersection_count += intersections.size();
      benchmark::DoNotOptimize(intersections.data());
      benchmark::DoNotOptimize(intersections.size());
    }
  }

  benchmark::DoNotOptimize(intersection_count);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(arc_pairs.size()));
}

BENCHMARK(BM_CoordinateToRadial)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_CoordinateFromRadial)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcFromEndpoints)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcCentralAngle)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcUnitNormal)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcIntersections)
    ->Arg(kSmallFixtureSize)
    ->Arg(kMediumFixtureSize)
    ->Arg(kLargeFixtureSize)
    ->Unit(benchmark::kMicrosecond);

} // namespace
