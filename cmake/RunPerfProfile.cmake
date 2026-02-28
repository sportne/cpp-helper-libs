if(NOT DEFINED PROFILE_BENCHMARK_FILTER)
  set(PROFILE_BENCHMARK_FILTER "^BM_MinorArcIntersections(/.*)?$")
endif()

if(NOT DEFINED BENCHMARK_COMMON_ARGS)
  set(BENCHMARK_COMMON_ARGS
    --benchmark_repetitions=1
    --benchmark_min_time=1.00s
  )
endif()

function(require_non_empty variable_name)
  if(NOT DEFINED ${variable_name} OR "${${variable_name}}" STREQUAL "")
    message(FATAL_ERROR "[bench-profile] Required variable '${variable_name}' is not set")
  endif()
endfunction()

require_non_empty(BENCHMARK_EXECUTABLE)
require_non_empty(OUTPUT_DIR)

if(NOT EXISTS "${BENCHMARK_EXECUTABLE}")
  message(FATAL_ERROR "[bench-profile] Benchmark executable not found: ${BENCHMARK_EXECUTABLE}")
endif()

find_program(PERF_EXECUTABLE NAMES perf)
if(NOT PERF_EXECUTABLE)
  message(FATAL_ERROR
    "[bench-profile] Linux perf is required for this target. "
    "Install perf and retry.")
endif()

if(NOT DEFINED PERF_DATA_PATH)
  set(PERF_DATA_PATH "${OUTPUT_DIR}/perf.data")
endif()

if(NOT DEFINED PERF_REPORT_PATH)
  set(PERF_REPORT_PATH "${OUTPUT_DIR}/perf-report.txt")
endif()

set(profile_log_path "${OUTPUT_DIR}/perf-profile-run.log")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

execute_process(
  COMMAND "${PERF_EXECUTABLE}" record
          --call-graph=dwarf
          --freq=999
          --output "${PERF_DATA_PATH}"
          "${BENCHMARK_EXECUTABLE}"
          --benchmark_filter=${PROFILE_BENCHMARK_FILTER}
          ${BENCHMARK_COMMON_ARGS}
          --benchmark_report_aggregates_only=true
          --benchmark_display_aggregates_only=true
  RESULT_VARIABLE record_result
  OUTPUT_VARIABLE record_stdout
  ERROR_VARIABLE record_stderr
)

file(WRITE "${profile_log_path}"
  "## perf record stdout\n${record_stdout}\n\n"
  "## perf record stderr\n${record_stderr}\n")

if(NOT record_result EQUAL 0)
  message(FATAL_ERROR
    "[bench-profile] perf record failed. "
    "Inspect ${profile_log_path}. "
    "On Linux you may need to lower kernel.perf_event_paranoid.")
endif()

execute_process(
  COMMAND "${PERF_EXECUTABLE}" report
          --stdio
          --input "${PERF_DATA_PATH}"
          --sort comm,dso,symbol
  RESULT_VARIABLE report_result
  OUTPUT_VARIABLE report_stdout
  ERROR_VARIABLE report_stderr
)

if(NOT report_result EQUAL 0)
  message(FATAL_ERROR
    "[bench-profile] perf report failed for ${PERF_DATA_PATH}.\n"
    "stderr:\n${report_stderr}")
endif()

file(WRITE "${PERF_REPORT_PATH}" "${report_stdout}")
message(STATUS "[bench-profile] benchmark filter: ${PROFILE_BENCHMARK_FILTER}")
message(STATUS "[bench-profile] wrote perf data: ${PERF_DATA_PATH}")
message(STATUS "[bench-profile] wrote perf report: ${PERF_REPORT_PATH}")
message(STATUS "[bench-profile] wrote run log: ${profile_log_path}")
