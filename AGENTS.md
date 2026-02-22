# AGENTS Guide

This file defines repository-wide expectations for AI coding agents and automation helpers.

## Scope
- Applies to the entire repository.

## Rules
- Prefer adding new functionality as isolated modules under `libs/<module>`.
- Require tests for behavior changes and new public APIs.
- Update documentation in `docs/` when modules, APIs, or workflows change.
- Avoid broad refactors unless explicitly requested.
- Keep CMake target-based (`target_*`) and avoid global compile flag mutations when possible.
- Keep local validation aligned with CI gates: format, static analysis, test matrix, and coverage threshold.
- Assume warnings-as-errors are enabled (`CPPHL_WARNINGS_AS_ERRORS=ON`) for first-party targets.
- Assume `_FORTIFY_SOURCE=2` is enabled for GNU/Clang `Release`, `RelWithDebInfo`, and `MinSizeRel` builds.

## Quality Gates
- Formatting: `cmake --build --preset build-clang-debug --target format-check`
- Static analysis: `cmake --preset clang-static-analysis` then `cmake --build --preset build-clang-static-analysis`
- Test matrix:
  - `ctest --preset test-clang-debug`
  - `ctest --preset test-clang-release`
  - `ctest --preset test-clang-debug-asan-ubsan`
- Coverage gate:
  - `cmake --preset gcc-coverage`
  - `cmake --build --preset build-gcc-coverage`
  - `ctest --preset test-gcc-coverage`
  - `gcovr --root . --filter '^libs/' --exclude '^third_party/' --exclude '.*/tests/.*' --fail-under-line 80 --txt --xml-pretty --xml coverage.xml`

For one-command parity with CI, use:
- `cmake --preset clang-debug`
- `cmake --build --preset build-clang-debug --target ci-local`
- `cmake --workflow --preset ci-local`
- `make ci`

## Standard Workflow
1. Locate or create the target module under `libs/<module>`.
2. Add or update public headers under `include/` and implementations under `src/`.
3. Add or update unit tests in `libs/<module>/tests/`.
4. Run local quality gates using `ci-local` (or run the gate commands above individually).
5. Update relevant docs in `docs/` and top-level `README.md` when behavior changes.
