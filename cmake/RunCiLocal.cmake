if(NOT DEFINED REPO_ROOT)
  # Default to repository root (one directory up from this script file).
  get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

# Required executables for running CTest and coverage reporting.
find_program(CTEST_EXECUTABLE NAMES ctest REQUIRED)
find_program(GCOVR_EXECUTABLE NAMES gcovr REQUIRED)

function(run_step)
  # Execute a command and stop immediately if it fails.
  execute_process(
    COMMAND ${ARGN}
    WORKING_DIRECTORY "${REPO_ROOT}"
    RESULT_VARIABLE run_result
  )
  if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "Step failed (${run_result}): ${ARGN}")
  endif()
endfunction()

function(clean_coverage_artifacts)
  # Remove stale coverage outputs from previous source layouts (for example,
  # renamed/deleted test files) so gcovr does not try to resolve missing paths.
  # Keep .gcno files because gcov needs them as notes for matching .gcda data.
  file(GLOB_RECURSE stale_gcda "${REPO_ROOT}/build/gcc-coverage/*.gcda")
  file(GLOB_RECURSE stale_gcov "${REPO_ROOT}/build/gcc-coverage/*.gcov")
  set(stale_coverage_files ${stale_gcda} ${stale_gcov})
  list(LENGTH stale_coverage_files stale_count)
  if(stale_count GREATER 0)
    file(REMOVE ${stale_coverage_files})
    message(STATUS "[ci-local] removed ${stale_count} stale coverage files")
  endif()
endfunction()

function(prune_test_coverage_artifacts)
  # gcovr can fail while parsing test translation-unit artifacts before exclude
  # filters apply. Remove test-only coverage artifacts because the coverage gate
  # targets first-party library code under libs/ (tests are excluded).
  file(GLOB_RECURSE all_cov_files
    "${REPO_ROOT}/build/gcc-coverage/*.gcda"
    "${REPO_ROOT}/build/gcc-coverage/*.gcno"
  )

  set(test_cov_files)
  foreach(cov_file IN LISTS all_cov_files)
    if(cov_file MATCHES "/tests/")
      list(APPEND test_cov_files "${cov_file}")
    endif()
  endforeach()

  list(LENGTH test_cov_files test_cov_count)
  if(test_cov_count GREATER 0)
    file(REMOVE ${test_cov_files})
    message(STATUS "[ci-local] pruned ${test_cov_count} test coverage files")
  endif()
endfunction()

# 1) Debug build: format gate + tests.
message(STATUS "[ci-local] clang-debug: configure/build/format-check/test")
run_step("${CMAKE_COMMAND}" --preset clang-debug)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-debug --parallel)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-debug --target format-check --parallel)
run_step("${CTEST_EXECUTABLE}" --preset test-clang-debug)

# 2) Static analysis build.
message(STATUS "[ci-local] clang-static-analysis: configure/build")
run_step("${CMAKE_COMMAND}" --preset clang-static-analysis)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-static-analysis --parallel)

# 3) Release build + tests.
message(STATUS "[ci-local] clang-release: configure/build/test")
run_step("${CMAKE_COMMAND}" --preset clang-release)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-release --parallel)
run_step("${CTEST_EXECUTABLE}" --preset test-clang-release)

# 4) Sanitizer build + tests. Leak detection is disabled for compatibility in
# some local environments where LeakSanitizer cannot run under ptrace.
message(STATUS "[ci-local] clang-debug-asan-ubsan: configure/build/test")
run_step("${CMAKE_COMMAND}" --preset clang-debug-asan-ubsan)
run_step("${CMAKE_COMMAND}" -E env ASAN_OPTIONS=detect_leaks=0 "${CMAKE_COMMAND}" --build --preset build-clang-debug-asan-ubsan --parallel)
run_step("${CMAKE_COMMAND}" -E env ASAN_OPTIONS=detect_leaks=0 "${CTEST_EXECUTABLE}" --preset test-clang-debug-asan-ubsan)

# 5) Coverage build + tests + threshold check.
message(STATUS "[ci-local] gcc-coverage: configure/build/test/coverage gate")
# Start coverage from a fresh build tree to avoid stale gcov artifacts after
# source/test file renames.
run_step("${CMAKE_COMMAND}" -E rm -rf "${REPO_ROOT}/build/gcc-coverage")
run_step("${CMAKE_COMMAND}" --preset gcc-coverage)
run_step("${CMAKE_COMMAND}" --build --preset build-gcc-coverage --parallel)
clean_coverage_artifacts()
run_step("${CTEST_EXECUTABLE}" --preset test-gcc-coverage)
prune_test_coverage_artifacts()
run_step(
  "${GCOVR_EXECUTABLE}"
  --root .
  --filter ^libs/
  --exclude ^third_party/
  --exclude .*/tests/.*
  --fail-under-line 80
  --txt
  --xml-pretty
  --xml coverage.xml
)

message(STATUS "[ci-local] complete")
