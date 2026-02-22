# Style Guide

## C++
- Use C++20.
- Favor small, focused headers and implementation files.
- Use namespaces rooted under `cpp_helper_libs`.
- Keep APIs simple and explicit.

## CMake
- Prefer target-based CMake (`target_link_libraries`, `target_include_directories`).
- Avoid global compile options unless absolutely necessary.
- Use interface targets for shared warnings and options.

## Testing
- Use GoogleTest for unit and smoke tests.
- Keep tests deterministic and fast.
- Group tests by module and add integration/smoke tests in `tests/`.
