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

## Standard Workflow
1. Locate or create the target module under `libs/<module>`.
2. Add or update public headers under `include/` and implementations under `src/`.
3. Add or update unit tests in `libs/<module>/tests/`.
4. Run configure, build, and tests.
5. Update relevant docs in `docs/` and top-level `README.md` when behavior changes.
