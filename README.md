# cpp-helper-libs

Modular C++ helper libraries built with CMake presets and target-based project configuration.

The repository currently contains first-party modules:
- `libs/math`: arithmetic helpers (`add`, `sub`)
- `libs/quantities`: strongly-typed measurable quantities with unit conversions
- `libs/linear_algebra`: immutable vector and matrix helpers (`Vector3`, `UnitVector3`, `Matrix3`,
  `Matrix`)
- `libs/spherical_geometry`: coordinate, ray, curve, and shape utilities on the unit sphere

Each module is built as both:
- static library target: `cpphl_<module>` (alias: `cpphl::<module>`)
- shared library target: `cpphl_<module>_shared` (alias: `cpphl::<module>_shared`)

## Quick Start

1. Initialize submodules (required for GoogleTest and Google Benchmark):

```bash
git submodule update --init --recursive
```

2. Configure, build, and test debug:

```bash
cmake --workflow --preset debug
```

Equivalent shortcut:

```bash
make debug
```

## Public API Example

```cpp
#include "cpp_helper_libs/linear_algebra/linear_algebra.hpp"
#include "cpp_helper_libs/math/arithmetic.hpp"
#include "cpp_helper_libs/quantities/quantities.hpp"
#include "cpp_helper_libs/spherical_geometry/spherical_geometry.hpp"

const int sum = cpp_helper_libs::math::add(2, 3); // 5
const int diff = cpp_helper_libs::math::sub(7, 4); // 3

const auto one_mile = cpp_helper_libs::quantities::Length::miles(1.0);
const auto meters = one_mile.in(cpp_helper_libs::quantities::Length::Unit::Meter); // 1609.344

const auto normal = cpp_helper_libs::linear_algebra::unit_normal(
    cpp_helper_libs::linear_algebra::Vector3(1.0, 0.0, 0.0),
    cpp_helper_libs::linear_algebra::Vector3(0.0, 1.0, 0.0));
// normal has value (0, 0, 1)

const cpp_helper_libs::linear_algebra::Matrix3 a(4.0, 1.0, 2.0,
                                                  1.0, 5.0, 1.0,
                                                  2.0, 1.0, 3.0);
const auto x = a.solve(cpp_helper_libs::linear_algebra::Vector3(7.0, 8.0, 5.0));
const double det = a.determinant(); // 37.0

const auto lon_lat = cpp_helper_libs::spherical_geometry::Coordinate::degrees(40.0, -74.0);
const auto radial = lon_lat.to_radial();
const auto equator_quarter = cpp_helper_libs::spherical_geometry::MinorArc::from_endpoints(
    cpp_helper_libs::linear_algebra::UnitVector3::from_components(1.0, 0.0, 0.0).value(),
    cpp_helper_libs::linear_algebra::UnitVector3::from_components(0.0, 1.0, 0.0).value());
```

## Repository Layout

```text
.
├── cmake/                 # Shared CMake modules and CI helper scripts
├── docs/                  # Contributor and workflow documentation
├── libs/                  # First-party helper library modules
│   └── math/
│   └── linear_algebra/
│   └── quantities/
│   └── spherical_geometry/
├── tests/                 # Cross-module smoke/integration tests
└── third_party/           # Vendored dependencies (GoogleTest/Google Benchmark submodules)
```

## Toolchain Prerequisites

- CMake 3.25+
- Ninja
- Clang/Clang++
- GCC/G++
- Git
- clang-format
- clang-tidy
- cppcheck
- include-what-you-use
- gcovr
- PMD (optional, for CPD duplicate-code scans)

## Common Commands

Use CMake workflow presets directly:

```bash
cmake --workflow --preset debug
cmake --workflow --preset release
cmake --workflow --preset asan
cmake --workflow --preset coverage
cmake --workflow --preset benchmark
cmake --workflow --preset benchmark-baseline
cmake --workflow --preset benchmark-compare
cmake --workflow --preset static-analysis
cmake --workflow --preset format-check
cmake --workflow --preset format
cmake --workflow --preset cpd
cmake --workflow --preset ci-local
```

Or use `make` wrappers:

```bash
make debug
make release
make asan
make coverage
make bench
make bench-baseline
make bench-compare
make static-analysis
make format-check
make format
make cpd
make ci
```

## Quality Gates

Local checks aligned with CI:

1. Formatting:
```bash
cmake --build --preset build-clang-debug --target format-check
```
2. Static analysis:
```bash
cmake --preset clang-static-analysis
cmake --build --preset build-clang-static-analysis
```
This preset runs `clang-tidy`, `cppcheck`, and `include-what-you-use`.
3. Test matrix:
```bash
ctest --preset test-clang-debug
ctest --preset test-clang-release
ctest --preset test-clang-debug-asan-ubsan
```
4. Duplicate-code scan (report-only by default):
```bash
cmake --workflow --preset cpd
```
The scan writes `build/cpd/cpd-report.md`. If PMD is not available, the step is skipped with a warning.
5. Coverage threshold (`>= 80%` for `libs/`, tests excluded):
```bash
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage
ctest --preset test-gcc-coverage
gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml
```

Single command parity with CI:

```bash
cmake --workflow --preset ci-local
```

## Local Performance Evaluation

Spherical-geometry benchmarks are available as a local workflow:

```bash
cmake --workflow --preset benchmark
cmake --workflow --preset benchmark-baseline
cmake --workflow --preset benchmark-compare
```

Equivalent `make` aliases:

```bash
make bench
make bench-baseline
make bench-compare
```

Outputs:
- latest run JSON: `build/clang-benchmark/benchmarks/spherical_geometry/latest.json`
- local baseline JSON: `docs/benchmarks/spherical_geometry-baseline.local.json`

`benchmark-compare` fails if any median `cpu_time` regresses by more than `10%`.

## Build Policy

- Language level: C++20
- Warning policy: first-party targets build with warnings-as-errors by default (`CPPHL_WARNINGS_AS_ERRORS=ON`)
- Fortify policy: `_FORTIFY_SOURCE=2` for GNU/Clang `Release`, `RelWithDebInfo`, and `MinSizeRel`
- Sanitizer policy: ASan/UBSan enabled only in sanitizer preset (`clang-debug-asan-ubsan`)

## Documentation

- `docs/README.md`
- `docs/build-and-test.md`
- `docs/architecture.md`
- `docs/contributing.md`
- `docs/linear-algebra.md`
- `docs/performance-benchmarks.md`
- `docs/quantities.md`
- `docs/spherical-geometry.md`
- `docs/style-guide.md`

## Adding a Module

1. Create `libs/<module>/include`, `libs/<module>/src`, and `libs/<module>/tests`.
2. Define both static/shared targets (`cpphl_<module>`, `cpphl_<module>_shared`) and aliases (`cpphl::<module>`, `cpphl::<module>_shared`).
3. Register the module in `libs/CMakeLists.txt`.
4. Add unit tests and register them with `gtest_discover_tests`.
5. Update `README.md` and relevant files under `docs/`.
