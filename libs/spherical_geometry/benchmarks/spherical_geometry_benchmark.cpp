// Copyright (c) 2026 sportne
// SPDX-License-Identifier: MIT

#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "cpp_helper_libs/quantities/angle.hpp"
#include "cpp_helper_libs/spherical_geometry/coordinate.hpp"
#include "cpp_helper_libs/spherical_geometry/minor_arc.hpp"

namespace {

using cpp_helper_libs::linear_algebra::UnitVector3;
using cpp_helper_libs::spherical_geometry::Coordinate;
using cpp_helper_libs::spherical_geometry::MinorArc;

constexpr std::size_t kCoordinateFixtureSize = 768U;
constexpr std::size_t kArcEndpointFixtureSize = 512U;
constexpr std::size_t kArcIntersectionFixtureSize = 256U;

std::vector<Coordinate> build_coordinate_fixture() {
  std::vector<Coordinate> coordinates;
  coordinates.reserve(kCoordinateFixtureSize);

  for (std::size_t index = 0U; index < kCoordinateFixtureSize; ++index) {
    const double latitude = -80.0 + static_cast<double>(index % 161U);
    const double longitude = -170.0 + static_cast<double>((index * 37U) % 341U);
    coordinates.push_back(Coordinate::degrees(latitude, longitude));
  }

  return coordinates;
}

std::vector<UnitVector3> build_radial_fixture() {
  const std::vector<Coordinate> coordinates = build_coordinate_fixture();
  std::vector<UnitVector3> radials;
  radials.reserve(coordinates.size());

  for (const Coordinate &coordinate : coordinates) {
    radials.push_back(coordinate.to_radial());
  }

  return radials;
}

std::vector<std::pair<UnitVector3, UnitVector3>> build_minor_arc_endpoint_fixture() {
  std::vector<std::pair<UnitVector3, UnitVector3>> endpoints;
  endpoints.reserve(kArcEndpointFixtureSize);

  for (std::size_t index = 0U; index < kArcEndpointFixtureSize; ++index) {
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

std::vector<ArcPair> build_minor_arc_intersection_fixture() {
  std::vector<ArcPair> pairs;
  pairs.reserve(kArcIntersectionFixtureSize);

  for (std::size_t index = 0U; index < kArcIntersectionFixtureSize; ++index) {
    const double base_longitude = -160.0 + static_cast<double>((index * 13U) % 321U);

    const UnitVector3 first_start = Coordinate::degrees(-30.0, base_longitude).to_radial();
    const UnitVector3 first_end = Coordinate::degrees(30.0, base_longitude + 70.0).to_radial();
    const UnitVector3 second_start = Coordinate::degrees(-30.0, base_longitude + 35.0).to_radial();
    const UnitVector3 second_end = Coordinate::degrees(30.0, base_longitude - 35.0).to_radial();

    const std::optional<MinorArc> first_arc = MinorArc::from_endpoints(first_start, first_end);
    const std::optional<MinorArc> second_arc =
        MinorArc::from_endpoints(second_start, second_end);
    if (!first_arc.has_value() || !second_arc.has_value()) {
      continue;
    }

    pairs.push_back(ArcPair{.first = first_arc.value(), .second = second_arc.value()});
  }

  return pairs;
}

void BM_CoordinateToRadial(benchmark::State &state) {
  static const std::vector<Coordinate> kCoordinates = build_coordinate_fixture();
  double checksum = 0.0;

  for (auto _ : state) {
    for (const Coordinate &coordinate : kCoordinates) {
      const UnitVector3 radial = coordinate.to_radial();
      checksum += radial.x();
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(kCoordinates.size()));
}

void BM_CoordinateFromRadial(benchmark::State &state) {
  static const std::vector<UnitVector3> kRadials = build_radial_fixture();
  double checksum = 0.0;

  for (auto _ : state) {
    for (const UnitVector3 &radial : kRadials) {
      const Coordinate coordinate = Coordinate::from_radial(radial);
      checksum += coordinate.latitude().in(cpp_helper_libs::quantities::Angle::Unit::Radian);
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(kRadials.size()));
}

void BM_MinorArcFromEndpoints(benchmark::State &state) {
  static const std::vector<std::pair<UnitVector3, UnitVector3>> kEndpoints =
      build_minor_arc_endpoint_fixture();
  std::size_t arcs_built = 0U;

  for (auto _ : state) {
    for (const auto &endpoint_pair : kEndpoints) {
      std::optional<MinorArc> arc = MinorArc::from_endpoints(endpoint_pair.first, endpoint_pair.second);
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
                          static_cast<std::int64_t>(kEndpoints.size()));
}

void BM_MinorArcIntersections(benchmark::State &state) {
  static const std::vector<ArcPair> kArcPairs = build_minor_arc_intersection_fixture();
  if (kArcPairs.empty()) {
    state.SkipWithError("No valid MinorArc fixtures available for intersections benchmark");
    return;
  }

  std::size_t intersection_count = 0U;

  for (auto _ : state) {
    for (const ArcPair &pair : kArcPairs) {
      std::vector<cpp_helper_libs::spherical_geometry::CurveIntersection> intersections =
          pair.first.intersections_with(pair.second);
      intersection_count += intersections.size();
      benchmark::DoNotOptimize(intersections.data());
      benchmark::DoNotOptimize(intersections.size());
    }
  }

  benchmark::DoNotOptimize(intersection_count);
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                          static_cast<std::int64_t>(kArcPairs.size()));
}

BENCHMARK(BM_CoordinateToRadial)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_CoordinateFromRadial)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcFromEndpoints)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_MinorArcIntersections)->Unit(benchmark::kMicrosecond);

} // namespace
