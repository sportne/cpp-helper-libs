# Build and Test

## Initialize Submodules

```bash
git submodule update --init --recursive
```

## Configure

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

## One-Command Local CI

```bash
cmake --workflow --preset ci-local
```

Equivalent alias:

```bash
make ci
```

## Shortcut Commands

You can use workflow presets directly:

```bash
cmake --workflow --preset debug
cmake --workflow --preset release
cmake --workflow --preset asan
cmake --workflow --preset static-analysis
cmake --workflow --preset format-check
cmake --workflow --preset format
```

Or use `Makefile` aliases:

```bash
make debug
make release
make asan
make static-analysis
make format-check
make format
make clean
make clean-all
make distclean
```

## Optional Sanitizers

```bash
cmake --preset clang-debug-asan-ubsan
cmake --build --preset build-clang-debug-asan-ubsan
ctest --preset test-clang-debug-asan-ubsan
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

This preset enables both `clang-tidy` and `cppcheck` and fails on findings.

## Warning Policy

- Presets enable warnings and treat them as errors on first-party targets (`CPPHL_WARNINGS_AS_ERRORS=ON`).
- To temporarily disable warnings-as-errors in a local build:

```bash
cmake --preset clang-debug -DCPPHL_WARNINGS_AS_ERRORS=OFF
cmake --build --preset build-clang-debug
```

- GNU/Clang `Release`, `RelWithDebInfo`, and `MinSizeRel` builds define `_FORTIFY_SOURCE=2`.
- Debug builds do not define `_FORTIFY_SOURCE`.

## Coverage

```bash
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage
ctest --preset test-gcc-coverage
gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml
```

Coverage must remain at or above `80%` line coverage for first-party code under `libs/`.

## CI Parity Matrix

The CI build/test job runs these preset triplets:
- `clang-debug` / `build-clang-debug` / `test-clang-debug`
- `clang-release` / `build-clang-release` / `test-clang-release`
- `clang-debug-asan-ubsan` / `build-clang-debug-asan-ubsan` / `test-clang-debug-asan-ubsan`

If configuration fails with a GoogleTest error, initialize submodules and retry.
