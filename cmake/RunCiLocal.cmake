if(NOT DEFINED REPO_ROOT)
  get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

find_program(CTEST_EXECUTABLE NAMES ctest REQUIRED)
find_program(GCOVR_EXECUTABLE NAMES gcovr REQUIRED)

function(run_step)
  execute_process(
    COMMAND ${ARGN}
    WORKING_DIRECTORY "${REPO_ROOT}"
    RESULT_VARIABLE run_result
  )
  if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "Step failed (${run_result}): ${ARGN}")
  endif()
endfunction()

message(STATUS "[ci-local] clang-debug: configure/build/format-check/test")
run_step("${CMAKE_COMMAND}" --preset clang-debug)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-debug --parallel)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-debug --target format-check --parallel)
run_step("${CTEST_EXECUTABLE}" --preset test-clang-debug)

message(STATUS "[ci-local] clang-static-analysis: configure/build")
run_step("${CMAKE_COMMAND}" --preset clang-static-analysis)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-static-analysis --parallel)

message(STATUS "[ci-local] clang-release: configure/build/test")
run_step("${CMAKE_COMMAND}" --preset clang-release)
run_step("${CMAKE_COMMAND}" --build --preset build-clang-release --parallel)
run_step("${CTEST_EXECUTABLE}" --preset test-clang-release)

message(STATUS "[ci-local] clang-debug-asan-ubsan: configure/build/test")
run_step("${CMAKE_COMMAND}" --preset clang-debug-asan-ubsan)
run_step("${CMAKE_COMMAND}" -E env ASAN_OPTIONS=detect_leaks=0 "${CMAKE_COMMAND}" --build --preset build-clang-debug-asan-ubsan --parallel)
run_step("${CMAKE_COMMAND}" -E env ASAN_OPTIONS=detect_leaks=0 "${CTEST_EXECUTABLE}" --preset test-clang-debug-asan-ubsan)

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
