#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "${repo_root}"

echo "[ci-local] clang-debug: configure/build/format-check/test"
cmake --preset clang-debug
cmake --build --preset build-clang-debug --parallel
cmake --build --preset build-clang-debug --target format-check --parallel
ctest --preset test-clang-debug

echo "[ci-local] clang-static-analysis: configure/build"
cmake --preset clang-static-analysis
cmake --build --preset build-clang-static-analysis --parallel

echo "[ci-local] clang-release: configure/build/test"
cmake --preset clang-release
cmake --build --preset build-clang-release --parallel
ctest --preset test-clang-release

echo "[ci-local] clang-debug-asan-ubsan: configure/build/test"
cmake --preset clang-debug-asan-ubsan
cmake --build --preset build-clang-debug-asan-ubsan --parallel
ctest --preset test-clang-debug-asan-ubsan

echo "[ci-local] gcc-coverage: configure/build/test/coverage gate"
cmake --preset gcc-coverage
cmake --build --preset build-gcc-coverage --parallel
ctest --preset test-gcc-coverage
gcovr \
  --root . \
  --filter '^libs/' \
  --exclude '^third_party/' \
  --exclude '.*/tests/.*' \
  --fail-under-line 80 \
  --txt \
  --xml-pretty \
  --xml coverage.xml

echo "[ci-local] complete"
