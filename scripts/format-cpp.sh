#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

if ! command -v clang-format >/dev/null 2>&1; then
  echo "clang-format is required but was not found on PATH." >&2
  exit 1
fi

files=()
while IFS= read -r -d '' path; do
  case "$path" in
    *.h|*.hh|*.hpp|*.c|*.cc|*.cpp|*.cxx)
      files+=("${repo_root}/${path}")
      ;;
  esac
done < <(git -C "${repo_root}" ls-files -z libs tests)

if [ "${#files[@]}" -eq 0 ]; then
  echo "No C/C++ files found under libs/ or tests/."
  exit 0
fi

clang-format -i "${files[@]}"
echo "Formatted ${#files[@]} C/C++ files."
