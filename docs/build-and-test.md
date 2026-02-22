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
./scripts/run-ci-local.sh
```

Alternative via CMake:

```bash
cmake --preset clang-debug
cmake --build --preset build-clang-debug --target ci-local
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
