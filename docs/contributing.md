# Contributing

## Expectations
- Keep changes scoped and module-focused.
- Add or update tests for behavior changes.
- Update docs in `docs/` and `README.md` for visible workflow/API changes.
- Ensure CI quality gates pass: format check, static analysis, test matrix, and coverage threshold.

## Typical Change Flow
1. Create/update code in the relevant `libs/<module>` area.
2. Add/update tests in `libs/<module>/tests` and/or `tests/`.
3. Configure and build using CMake presets.
4. Run format and static analysis checks.
5. Run tests with CTest.
6. Validate coverage for library code (`>= 80%` line coverage).
7. Open a PR with a summary of behavior and tests.
