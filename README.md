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
- Ninja (optional, but used by presets)
- Git (for submodules)

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

## Adding a New Module
1. Create `libs/<module>/include`, `libs/<module>/src`, and `libs/<module>/tests`.
2. Add a `libs/<module>/CMakeLists.txt` with a library target and alias (`cpphl::<module>`).
3. Register the module in `libs/CMakeLists.txt`.
4. Add unit tests with `gtest_discover_tests(...)`.
5. Update `docs/architecture.md` and related documentation.
