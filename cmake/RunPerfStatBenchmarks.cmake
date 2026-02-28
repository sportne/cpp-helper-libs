if(POLICY CMP0007)
  cmake_policy(SET CMP0007 NEW)
endif()

if(NOT DEFINED PERF_EVENTS)
  set(PERF_EVENTS "cycles,instructions,cache-misses,branch-misses")
endif()

if(NOT DEFINED PERF_FALLBACK_EVENTS)
  # Software counters are broadly available and work on constrained/virtualized
  # environments where hardware PMU counters may be unsupported.
  set(PERF_FALLBACK_EVENTS "cpu-clock,task-clock,context-switches,page-faults")
endif()

if(NOT DEFINED BENCHMARK_COMMON_ARGS)
  set(BENCHMARK_COMMON_ARGS
    --benchmark_repetitions=5
    --benchmark_min_time=0.30s
  )
endif()

function(require_non_empty variable_name)
  if(NOT DEFINED ${variable_name} OR "${${variable_name}}" STREQUAL "")
    message(FATAL_ERROR "[bench-perfstat] Required variable '${variable_name}' is not set")
  endif()
endfunction()

require_non_empty(BENCHMARK_EXECUTABLE)
require_non_empty(OUTPUT_DIR)
require_non_empty(CSV_OUTPUT_PATH)

if(NOT EXISTS "${BENCHMARK_EXECUTABLE}")
  message(FATAL_ERROR "[bench-perfstat] Benchmark executable not found: ${BENCHMARK_EXECUTABLE}")
endif()

find_program(PERF_EXECUTABLE NAMES perf)
if(NOT PERF_EXECUTABLE)
  message(FATAL_ERROR
    "[bench-perfstat] Linux perf is required for this target. "
    "Install perf and retry.")
endif()

set(raw_output_dir "${OUTPUT_DIR}/raw")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")
file(MAKE_DIRECTORY "${raw_output_dir}")

execute_process(
  COMMAND "${BENCHMARK_EXECUTABLE}" --benchmark_list_tests=true
  RESULT_VARIABLE list_result
  OUTPUT_VARIABLE list_output
  ERROR_VARIABLE list_error
)
if(NOT list_result EQUAL 0)
  message(FATAL_ERROR
    "[bench-perfstat] Failed to list benchmark names.\n"
    "stdout:\n${list_output}\n"
    "stderr:\n${list_error}")
endif()

string(REPLACE "\r\n" "\n" list_output "${list_output}")
string(REPLACE "\n" ";" benchmark_lines "${list_output}")
string(REPLACE "," ";" perf_events_list "${PERF_EVENTS}")
string(REPLACE "," ";" perf_fallback_events_list "${PERF_FALLBACK_EVENTS}")

set(csv_lines "benchmark,event,value,unit")
set(benchmarks_processed 0)
set(total_data_rows 0)
foreach(candidate IN LISTS benchmark_lines)
  string(STRIP "${candidate}" benchmark_name)
  if(benchmark_name STREQUAL "")
    continue()
  endif()

  math(EXPR benchmarks_processed "${benchmarks_processed} + 1")
  string(REGEX REPLACE "[^A-Za-z0-9_.-]" "_" benchmark_stub "${benchmark_name}")
  set(raw_file_path "${raw_output_dir}/${benchmark_stub}.txt")

  set(perf_stderr "")
  execute_process(
    COMMAND "${PERF_EXECUTABLE}" stat
            -x,
            --no-big-num
            --event "${PERF_EVENTS}"
            "${BENCHMARK_EXECUTABLE}"
            --benchmark_filter=^${benchmark_name}$
            ${BENCHMARK_COMMON_ARGS}
            --benchmark_report_aggregates_only=true
            --benchmark_display_aggregates_only=true
    RESULT_VARIABLE perf_result
    OUTPUT_VARIABLE perf_stdout
    ERROR_VARIABLE perf_stderr
  )

  file(WRITE "${raw_file_path}" "### Benchmark: ${benchmark_name}\n\n")
  file(APPEND "${raw_file_path}" "## Attempt 1 events\n${PERF_EVENTS}\n\n")
  file(APPEND "${raw_file_path}" "## Attempt 1 stdout\n${perf_stdout}\n\n")
  file(APPEND "${raw_file_path}" "## Attempt 1 stderr\n${perf_stderr}\n\n")

  if(NOT perf_result EQUAL 0)
    message(FATAL_ERROR
      "[bench-perfstat] perf stat failed for benchmark '${benchmark_name}'. "
      "Inspect ${raw_file_path} for details.")
  endif()

  set(benchmark_data_rows 0)
  set(attempted_fallback FALSE)

  foreach(perf_event_set IN ITEMS "primary" "fallback")
    if(perf_event_set STREQUAL "fallback" AND benchmark_data_rows GREATER 0)
      break()
    endif()
    if(perf_event_set STREQUAL "fallback")
      set(attempted_fallback TRUE)
      set(active_events "${PERF_FALLBACK_EVENTS}")
      set(active_events_list "${perf_fallback_events_list}")

      execute_process(
        COMMAND "${PERF_EXECUTABLE}" stat
                -x,
                --no-big-num
                --event "${active_events}"
                "${BENCHMARK_EXECUTABLE}"
                --benchmark_filter=^${benchmark_name}$
                ${BENCHMARK_COMMON_ARGS}
                --benchmark_report_aggregates_only=true
                --benchmark_display_aggregates_only=true
        RESULT_VARIABLE perf_result
        OUTPUT_VARIABLE perf_stdout
        ERROR_VARIABLE perf_stderr
      )

      file(APPEND "${raw_file_path}" "## Attempt 2 events\n${active_events}\n\n")
      file(APPEND "${raw_file_path}" "## Attempt 2 stdout\n${perf_stdout}\n\n")
      file(APPEND "${raw_file_path}" "## Attempt 2 stderr\n${perf_stderr}\n\n")

      if(NOT perf_result EQUAL 0)
        message(FATAL_ERROR
          "[bench-perfstat] perf stat fallback failed for benchmark '${benchmark_name}'. "
          "Inspect ${raw_file_path} for details.")
      endif()
    else()
      set(active_events "${PERF_EVENTS}")
      set(active_events_list "${perf_events_list}")
    endif()

    string(REPLACE "\r\n" "\n" perf_stderr "${perf_stderr}")
    string(REPLACE "\n" ";" perf_lines "${perf_stderr}")

    foreach(perf_line IN LISTS perf_lines)
      string(STRIP "${perf_line}" perf_line)
      if(perf_line STREQUAL "" OR perf_line MATCHES "^#" OR NOT perf_line MATCHES ",")
        continue()
      endif()

      # Parse the first three CSV fields from perf stat -x, output:
      # value,unit,event,...
      string(REGEX MATCH "^([^,]*),([^,]*),([^,]*)," perf_match "${perf_line}")
      if(perf_match STREQUAL "")
        continue()
      endif()

      set(field_value "${CMAKE_MATCH_1}")
      set(field_unit "${CMAKE_MATCH_2}")
      set(field_event "${CMAKE_MATCH_3}")
      string(STRIP "${field_value}" field_value)
      string(STRIP "${field_unit}" field_unit)
      string(STRIP "${field_event}" field_event)

      # Ignore non-counter lines (for example benchmark banner text).
      set(event_is_requested FALSE)
      foreach(requested_event IN LISTS active_events_list)
        string(STRIP "${requested_event}" requested_event)
        if(requested_event STREQUAL "")
          continue()
        endif()
        if(field_event STREQUAL requested_event OR field_event MATCHES "^${requested_event}:")
          set(event_is_requested TRUE)
          break()
        endif()
      endforeach()
      if(NOT event_is_requested)
        continue()
      endif()

      if(field_value STREQUAL "" OR field_value STREQUAL "<not counted>" OR
         field_value STREQUAL "<not supported>")
        continue()
      endif()

      string(REPLACE "\"" "\"\"" escaped_benchmark_name "${benchmark_name}")
      list(APPEND csv_lines
        "\"${escaped_benchmark_name}\",${field_event},${field_value},${field_unit}")
      math(EXPR benchmark_data_rows "${benchmark_data_rows} + 1")
    endforeach()

    if(benchmark_data_rows EQUAL 0 AND perf_event_set STREQUAL "primary")
      message(STATUS
        "[bench-perfstat] '${benchmark_name}' produced no usable counters for "
        "hardware events; retrying with fallback software events: ${PERF_FALLBACK_EVENTS}")
    endif()
  endforeach()

  math(EXPR total_data_rows "${total_data_rows} + ${benchmark_data_rows}")
  if(benchmark_data_rows EQUAL 0)
    if(attempted_fallback)
      message(STATUS
        "[bench-perfstat] '${benchmark_name}' produced no usable counters even "
        "after fallback; see ${raw_file_path}")
    else()
      message(STATUS
        "[bench-perfstat] '${benchmark_name}' produced no usable counters; "
        "see ${raw_file_path}")
    endif()
  endif()
endforeach()

if(benchmarks_processed EQUAL 0)
  message(FATAL_ERROR "[bench-perfstat] No benchmarks were discovered.")
endif()
if(total_data_rows EQUAL 0)
  message(FATAL_ERROR
    "[bench-perfstat] No usable perf counters were collected across all benchmarks. "
    "Inspect ${raw_output_dir} and verify perf permissions/event availability.")
endif()

string(REPLACE ";" "\n" csv_body "${csv_lines}")
get_filename_component(csv_dir "${CSV_OUTPUT_PATH}" DIRECTORY)
file(MAKE_DIRECTORY "${csv_dir}")
file(WRITE "${CSV_OUTPUT_PATH}" "${csv_body}\n")

message(STATUS "[bench-perfstat] processed ${benchmarks_processed} benchmark(s)")
message(STATUS "[bench-perfstat] collected ${total_data_rows} counter row(s)")
message(STATUS "[bench-perfstat] wrote CSV: ${CSV_OUTPUT_PATH}")
message(STATUS "[bench-perfstat] wrote raw logs: ${raw_output_dir}")
