# Style Guide

## C++

- Use C++20.
- Prefer small, focused headers and implementation files.
- Use namespaces rooted under `cpp_helper_libs`.
- Keep public APIs simple and explicit.
- Keep includes stable and namespaced (for example, `cpp_helper_libs/math/arithmetic.hpp`).
- Write warning-clean code under strict warning profiles.

## CMake

- Prefer target-based CMake (`target_link_libraries`, `target_include_directories`).
- Avoid global compile option mutations when target-level wiring is possible.
- Use interface targets for shared behavior (`cpphl_project_options`, `cpphl_warnings`, `cpphl_sanitizers`, `cpphl_coverage`).
- Keep module target naming consistent:
  - static target: `cpphl_<module>`
  - shared target: `cpphl_<module>_shared`
  - alias targets: `cpphl::<module>`, `cpphl::<module>_shared`

## Testing

- Use GoogleTest for unit and smoke tests.
- Keep tests deterministic and fast.
- Keep module unit tests in `libs/<module>/tests`.
- Keep cross-module smoke/integration tests in `tests/`.
- Register tests through `gtest_discover_tests`.

## Build and Analysis Policy

- Treat warnings as errors on first-party targets (`CPPHL_WARNINGS_AS_ERRORS=ON`).
- Run static analysis with `clang-tidy` and `cppcheck` using the `clang-static-analysis` preset.
- For GNU/Clang optimized configurations, `_FORTIFY_SOURCE=2` is enabled.

## Documentation

- Update `README.md` and relevant files in `docs/` whenever APIs, modules, or workflows change.
- Keep examples and commands aligned with `CMakePresets.json` and `Makefile`.
