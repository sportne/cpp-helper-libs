# Architecture

## Module Boundaries
- Each helper library module lives under `libs/<module>`.
- Module public headers live in `libs/<module>/include`.
- Module implementations live in `libs/<module>/src`.
- Module unit tests live in `libs/<module>/tests`.

## Dependency Rules
- Prefer dependencies through explicit CMake targets.
- Keep modules decoupled unless a dependency is intentional and documented.
- Avoid cross-module include path hacks; expose required headers through target include directories.
