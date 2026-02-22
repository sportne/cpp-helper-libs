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
run_step("${CMAKE_COMMAND}" --preset gcc-coverage)
run_step("${CMAKE_COMMAND}" --build --preset build-gcc-coverage --parallel)
run_step("${CTEST_EXECUTABLE}" --preset test-gcc-coverage)
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
