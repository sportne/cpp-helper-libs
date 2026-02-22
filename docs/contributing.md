# Contributing

## Expectations
- Keep changes scoped and module-focused.
- Add or update tests for behavior changes.
- Update docs in `docs/` and `README.md` for visible workflow/API changes.
- Ensure CI quality gates pass: format check, static analysis, test matrix, and coverage threshold.
- Keep warning-clean code: first-party targets are built with warnings-as-errors by default.

## Typical Change Flow
1. Create/update code in the relevant `libs/<module>` area.
2. Add/update tests in `libs/<module>/tests` and/or `tests/`.
3. Configure and build using CMake presets (or shortcut commands like `make debug`).
4. Run format and static analysis checks.
5. Run tests with CTest.
6. Validate coverage for library code (`>= 80%` line coverage).
7. If needed during local iteration, temporarily disable `CPPHL_WARNINGS_AS_ERRORS`; re-enable before final validation.
8. Open a PR with a summary of behavior and tests.
