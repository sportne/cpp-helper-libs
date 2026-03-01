# Architecture

## Repository Topology

- First-party library code lives under `libs/`.
- Each module is isolated under `libs/<module>/`.
- Cross-module smoke/integration tests live under `tests/`.
- Third-party dependencies live under `third_party/` and are not part of first-party quality metrics.

## Module Structure

Each module should follow this layout:

```text
libs/<module>/
├── include/          # Public headers
├── src/              # Private implementation
├── tests/            # Module-local unit tests
└── CMakeLists.txt
```

## Target Conventions

- Static library target: `cpphl_<module>` (for example, `cpphl_math`).
- Shared library target: `cpphl_<module>_shared` (for example, `cpphl_math_shared`).
- Consumer-facing aliases: `cpphl::<module>` and `cpphl::<module>_shared`.
- Public headers should use namespaced include paths (for example, `cpp_helper_libs/math/arithmetic.hpp`).

## Dependency Model

- Prefer target-based dependency wiring (`target_link_libraries`, `target_include_directories`).
- Link shared behavior through interface targets:
  - `cpphl_project_options`
  - `cpphl_warnings`
  - `cpphl_sanitizers`
  - `cpphl_coverage`
- Avoid global compile option mutations when a target-scoped alternative exists.

## Testing Model

- Module unit tests are declared in `libs/<module>/tests`.
- Repository smoke tests are declared in `tests/`.
- Tests are registered with CTest via `gtest_discover_tests`.

## Current First-Party Modules

- `math`: simple integer arithmetic helpers (`add`, `sub`)
- `quantities`: strongly-typed measurable quantities with unit-safe conversions and comparisons
- `linear_algebra`: immutable 3D vectors and matrix math (`Matrix3` plus a basic dynamic `Matrix`).
  Advanced routines (SVD/eigendecomposition/pseudoinverse/reshape-slicing) are intentionally
  deferred to a later phase.
- `spherical_geometry`: spherical coordinate/ray primitives, curve types (`MinorArc`, `MajorArc`,
  `SmallArc`, `ZeroLengthCurve`), curve intersections, and spherical shapes (`SphericalPolygon`,
  `SphericalCircle`, `SphericalEllipse`).
- `path_finding`: geometry-agnostic A* search interfaces and solver implementation.
- `dubins_path_finding`: spherical Dubins-constrained planner layered on `path_finding` and
  `spherical_geometry`.
