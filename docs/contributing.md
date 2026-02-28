# Contributing

## Expectations

- Keep changes scoped and module-focused.
- Place new functionality under `libs/<module>`.
- Add or update tests for behavior changes and new public APIs.
- Update `README.md` and files in `docs/` when behavior, modules, or workflows change.
- Keep CMake target-based and avoid global compile flag mutations when possible.
- Keep warning-clean code; first-party targets use warnings-as-errors by default.
- Write beginner-friendly docs and comments for new APIs:
  - assume a high school senior contributor is reading the code for the first time
  - explain important domain terms and invariants
  - explain what non-obvious constants mean (units, thresholds, geometry meaning)
  - explain what each private field stores when the class is part of the public API surface

## Standard Change Flow

1. Implement code in the target module under `libs/<module>`.
2. Update public headers (`include/`) and implementation files (`src/`) as needed.
3. Add or update module unit tests in `libs/<module>/tests`.
4. Add or update smoke/integration tests in `tests/` when cross-module behavior changes.
5. Run quality gates locally.
6. Update docs and README for externally visible changes.
7. Open a PR with a focused summary and test evidence.

## Local Quality Gates

Formatting:

```bash
cmake --build --preset build-clang-debug --target format-check
```

Static analysis:

```bash
cmake --preset clang-static-analysis
cmake --build --preset build-clang-static-analysis
```

The static-analysis preset runs `clang-tidy`, `cppcheck`, and `include-what-you-use`.

Test matrix:

```bash
ctest --preset test-clang-debug
ctest --preset test-clang-release
ctest --preset test-clang-debug-asan-ubsan
```

Duplicate-code scan (report-only by default):

```bash
cmake --workflow --preset cpd
```

This generates `build/cpd/cpd-report.md`. If PMD is not installed, the scan is skipped with a warning.

Coverage:

```bash
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage
ctest --preset test-gcc-coverage
gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml
```

Optional local performance check (`spherical_geometry`):

```bash
cmake --workflow --preset benchmark
cmake --workflow --preset benchmark-baseline
cmake --workflow --preset benchmark-compare
```

Equivalent shortcuts:

```bash
make bench
make bench-baseline
make bench-compare
```

Single command CI parity:

```bash
cmake --workflow --preset ci-local
```

## Pull Request Checklist

- Scope is module-focused and avoids unrelated refactors.
- New/changed behavior has tests.
- Docs are updated where needed.
- New constants and private fields are documented with intent-level comments.
- Local CI-equivalent checks pass.
