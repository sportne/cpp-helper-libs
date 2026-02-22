# cpp-helper-libs

A modular C++ helper library workspace for small reusable components (math, geometry, and more), built with CMake and tuned for Clang-based workflows.

## Repository Layout

```text
.
├── AGENTS.md
├── cmake/
├── docs/
├── libs/
│   └── math/
├── tests/
└── third_party/
```

## Prerequisites
- CMake 3.25+
- Clang/Clang++
- GCC/G++
- Ninja (optional, but used by presets)
- Git (for submodules)
- clang-format, clang-tidy, cppcheck
- gcovr (for coverage report generation)

## Quick Start

1. Initialize dependencies:

```bash
git submodule update --init --recursive
```

2. Configure (Debug, Clang):

```bash
cmake --preset clang-debug
```

3. Build:

```bash
cmake --build --preset build-clang-debug
```

4. Run tests:

```bash
ctest --preset test-clang-debug
```

## Quality Gates

CI enforces the following checks on pull requests and `main` pushes:
- Formatting check with `clang-format`.
- Static analysis with `clang-tidy` and `cppcheck`.
- Build and test matrix (`clang-debug`, `clang-release`, `clang-debug-asan-ubsan`).
- Coverage gate: minimum `80%` line coverage over first-party library code under `libs/` (tests excluded).

### One-Command Local CI

```bash
cmake --preset clang-debug
cmake --build --preset build-clang-debug --target ci-local
```

Optional shell wrapper (Unix-like environments):

```bash
./scripts/run-ci-local.sh
```

### Local Formatting

```bash
cmake --preset clang-debug
cmake --build --preset build-clang-debug --target format-check
cmake --build --preset build-clang-debug --target format
```

### Local Static Analysis

```bash
cmake --preset clang-static-analysis
cmake --build --preset build-clang-static-analysis
```

### Local Coverage

```bash
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage
ctest --preset test-gcc-coverage
gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml
```

## Adding a New Module
1. Create `libs/<module>/include`, `libs/<module>/src`, and `libs/<module>/tests`.
2. Add a `libs/<module>/CMakeLists.txt` with a library target and alias (`cpphl::<module>`).
3. Register the module in `libs/CMakeLists.txt`.
4. Add unit tests with `gtest_discover_tests(...)`.
5. Update `docs/architecture.md` and related documentation.
