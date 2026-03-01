# Dubins Path Finding Module

## What This Module Is For

`libs/dubins_path_finding` solves spherical path planning for a forward-only,
minimum-turn-radius vehicle (Dubins-style motion).

It is built on top of:

- `libs/path_finding` (generic A*)
- `libs/spherical_geometry` (rays, arcs, circles, polygons)

Public umbrella include:

```cpp
#include "cpp_helper_libs/dubins_path_finding/dubins_path_finding.hpp"
```

## Problem Model

The planner assumes:

- movement happens on the surface of a sphere
- obstacles are spherical circles and spherical polygons
- obstacle interiors and boundaries are blocked
- path is a sequence of:
  - `MinorArc` (straight geodesic motion)
  - `SmallArc` (turning motion)
- objective is minimum path length over the lattice graph

## API Overview

Main declaration:

- `cpp_helper_libs/dubins_path_finding/spherical_dubins_planner.hpp`

Key types:

- `Request`: start/goal rays, environment, vehicle model, planner config
- `Result`: status, optional path, expanded-node count
- `Path`: segment sequence + angular and surface length

Planner entry point:

- `plan_spherical_dubins_path(const Request&)`

## Search Strategy

- Uses lattice A* (resolution-complete on the configured grid).
- Node identity is based on discretized latitude/longitude/heading bins.
- Each expansion generates exactly three forward primitives:
  - straight (`MinorArc`)
  - left turn (`SmallArc`)
  - right turn (`SmallArc`)
- Goal requires both:
  - position error within tolerance
  - heading error within tolerance

## Operating Envelope Assumption

This module assumes typical requests are local:

- start-to-goal central-angle separation is expected to be at most **50 degrees**

Requests outside this envelope are rejected as `InvalidInput`.

## Collision Policy

A candidate segment is rejected when any obstruction:

- contains the segment start point
- contains the segment end point
- contains the segment midpoint (`t = 0.5`)
- intersects segment boundary inclusively

## Where To Read Next

- Module CMake wiring: `libs/dubins_path_finding/CMakeLists.txt`
- Unit tests: `libs/dubins_path_finding/tests/spherical_dubins_planner_test.cpp`
- Generic A* base module: `docs/path-finding.md`
