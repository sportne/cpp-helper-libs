# Build and Test

## Prerequisites

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

## Initialize Submodules

```bash
git submodule update --init --recursive
```

GoogleTest and Google Benchmark are vendored as git submodules under:
- `third_party/googletest`
- `third_party/googlebenchmark`

## Fast Path

Configure, build, and test debug in one command:

```bash
cmake --workflow --preset debug
```

Equivalent shortcut:

```bash
make debug
```

## Configure (Explicit)

```bash
cmake --preset clang-debug
```

## Build

```bash
cmake --build --preset build-clang-debug
```

## Test

```bash
ctest --preset test-clang-debug
```

## Build/Test Matrix

Debug:

```bash
cmake --workflow --preset debug
```

Release:

```bash
cmake --workflow --preset release
```

ASan/UBSan:

```bash
cmake --workflow --preset asan
```

Coverage:

```bash
cmake --workflow --preset coverage
```

## One-Command Local CI

```bash
cmake --workflow --preset ci-local
```

Equivalent alias:

```bash
make ci
```

The `ci-local` target runs these gates in sequence:

1. `clang-debug` configure/build + CPD duplicate-code scan (`cpd`) + `format-check` + debug tests.
3. `clang-static-analysis` configure/build.
4. `clang-release` configure/build/test.
5. `clang-debug-asan-ubsan` configure/build/test.
6. `gcc-coverage` configure/build/test + coverage threshold check.

## Shortcut Commands

You can use workflow presets directly:

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
```

Or use `Makefile` aliases:

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
make clean
make clean-all
make distclean
```

## Formatting

```bash
cmake --preset clang-debug
cmake --build --preset build-clang-debug --target format-check
cmake --build --preset build-clang-debug --target format
```

## Static Analysis

```bash
cmake --preset clang-static-analysis
cmake --build --preset build-clang-static-analysis
```

This preset enables `clang-tidy`, `cppcheck`, and `include-what-you-use` and fails on findings.

## Duplicate-Code Scan (CPD)

```bash
cmake --workflow --preset cpd
```

Equivalent shortcut:

```bash
make cpd
```

Defaults:
- scans tracked first-party files under `libs/**/src` and `libs/**/include` (tests excluded)
- uses `minimum-tokens=220` with `--ignore-identifiers --ignore-literals`
- report path: `build/cpd/cpd-report.md`
- report-only mode (`CPPHL_CPD_FAIL_ON_VIOLATION=OFF`)

Tool resolution order:
1. `CPPHL_PMD_EXECUTABLE` cache path override
2. `pmd` / `pmd.bat` from `PATH`
3. fallback under `third_party/pmd/*/bin/`

If PMD is unavailable, CPD is skipped with a status warning and the command succeeds.

## Warning Policy

- Presets enable warnings and treat them as errors on first-party targets (`CPPHL_WARNINGS_AS_ERRORS=ON`).
- To temporarily disable warnings-as-errors in a local build:

```bash
cmake --preset clang-debug -DCPPHL_WARNINGS_AS_ERRORS=OFF
cmake --build --preset build-clang-debug
```

- GNU/Clang `Release`, `RelWithDebInfo`, and `MinSizeRel` builds define `_FORTIFY_SOURCE=2`.
- Debug builds do not define `_FORTIFY_SOURCE`.

## Optional Sanitizers

```bash
cmake --preset clang-debug-asan-ubsan
cmake --build --preset build-clang-debug-asan-ubsan
ctest --preset test-clang-debug-asan-ubsan
```

## Coverage

```bash
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage
ctest --preset test-gcc-coverage
gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml
```

Coverage must remain at or above `80%` line coverage for first-party code under `libs/`.

## Local Performance Benchmarks

Run spherical-geometry benchmarks and write JSON output:

```bash
cmake --workflow --preset benchmark
```

Record/update local baseline:

```bash
cmake --workflow --preset benchmark-baseline
```

Compare against local baseline (fails on >10% median `cpu_time` regressions):

```bash
cmake --workflow --preset benchmark-compare
```

Equivalent shortcuts:

```bash
make bench
make bench-baseline
make bench-compare
```

Files:
- latest run JSON: `build/clang-benchmark/benchmarks/spherical_geometry/latest.json`
- local baseline JSON: `docs/benchmarks/spherical_geometry-baseline.local.json`

## CI Parity Matrix

The CI build/test job runs these preset triplets:
- `clang-debug` / `build-clang-debug` / `test-clang-debug`
- `clang-release` / `build-clang-release` / `test-clang-release`
- `clang-debug-asan-ubsan` / `build-clang-debug-asan-ubsan` / `test-clang-debug-asan-ubsan`

If configuration fails with a GoogleTest error, initialize submodules and retry.
