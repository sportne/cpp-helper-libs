# Path Finding Module

## What This Module Is For

`libs/path_finding` provides a **geometry-agnostic A\*** search engine.

It does not know about spheres, Dubins cars, roads, or maps.
You provide those rules through a problem interface.

Public umbrella include:

```cpp
#include "cpp_helper_libs/path_finding/path_finding.hpp"
```

## Core API

The main public types are in:

- `cpp_helper_libs/path_finding/a_star.hpp`

Key pieces:

- `NodeId`: opaque node identifier (`std::uint64_t`)
- `WeightedEdge`: directed edge (`to`, `cost`, `payload`)
- `AStarProblem`: interface with `is_goal`, `heuristic`, `expand`
- `AStarConfig`: runtime controls (`max_expanded_nodes`)
- `AStarResult`: status + chosen node/payload path + total cost + expanded-node count

## Solver Behavior

- Uses min-heap A* with `f = g + h`.
- Tracks best known `g` per node.
- Reconstructs both:
  - node path (`node_path`)
  - payload path (`edge_payload_path`) for domain-specific edge metadata
- Ignores invalid edges defensively:
  - negative edge cost
  - non-finite edge cost
- Returns `InvalidInput` when config is invalid (for example, `max_expanded_nodes == 0`).

## Notes For Implementers

- Edge costs must be non-negative for shortest-path guarantees.
- Heuristic should be finite and preferably admissible for best A* behavior.
- Keep payload IDs stable if you want deterministic route reconstruction.

## Where To Read Next

- Module CMake wiring: `libs/path_finding/CMakeLists.txt`
- Unit tests: `libs/path_finding/tests/a_star_test.cpp`
- Dubins specialization built on this engine: `docs/dubins-path-finding.md`
