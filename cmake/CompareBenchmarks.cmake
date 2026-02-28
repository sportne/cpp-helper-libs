if(NOT DEFINED BENCHMARK_METRIC)
  set(BENCHMARK_METRIC "cpu_time")
endif()

if(NOT DEFINED BENCHMARK_AGGREGATE)
  set(BENCHMARK_AGGREGATE "median")
endif()

if(NOT DEFINED REGRESSION_THRESHOLD_PERCENT)
  set(REGRESSION_THRESHOLD_PERCENT 10)
endif()

function(require_non_empty variable_name)
  if(NOT DEFINED ${variable_name} OR "${${variable_name}}" STREQUAL "")
    message(FATAL_ERROR "[bench-compare] Required variable '${variable_name}' is not set")
  endif()
endfunction()

function(decimal_to_micro_units decimal_value output_variable)
  # Convert a plain decimal string (up to 6 fractional digits) into an integer
  # value scaled by 1,000,000 so threshold arithmetic can use integer math.
  if(NOT "${decimal_value}" MATCHES "^-?[0-9]+(\\.[0-9]+)?$")
    message(FATAL_ERROR
      "[bench-compare] Unsupported numeric value '${decimal_value}'. "
      "Expected a plain decimal number.")
  endif()

  set(raw_value "${decimal_value}")
  set(sign 1)
  if(raw_value MATCHES "^-")
    set(sign -1)
    string(SUBSTRING "${raw_value}" 1 -1 raw_value)
  endif()

  string(FIND "${raw_value}" "." decimal_point_index)
  if(decimal_point_index EQUAL -1)
    set(integer_part "${raw_value}")
    set(fraction_part "0")
  else()
    string(SUBSTRING "${raw_value}" 0 ${decimal_point_index} integer_part)
    math(EXPR fraction_start "${decimal_point_index} + 1")
    string(SUBSTRING "${raw_value}" ${fraction_start} -1 fraction_part)
  endif()

  if(integer_part STREQUAL "")
    set(integer_part "0")
  endif()

  if(fraction_part STREQUAL "")
    set(fraction_part "0")
  endif()

  string(LENGTH "${fraction_part}" fraction_length)
  if(fraction_length GREATER 6)
    string(SUBSTRING "${fraction_part}" 0 6 fraction_part)
  else()
    while(fraction_length LESS 6)
      string(APPEND fraction_part "0")
      math(EXPR fraction_length "${fraction_length} + 1")
    endwhile()
  endif()

  string(REGEX REPLACE "^0+" "" integer_part_trimmed "${integer_part}")
  if(integer_part_trimmed STREQUAL "")
    set(integer_part_trimmed "0")
  endif()

  string(REGEX REPLACE "^0+" "" fraction_part_trimmed "${fraction_part}")
  if(fraction_part_trimmed STREQUAL "")
    set(fraction_part_trimmed "0")
  endif()

  math(EXPR scaled_value "${integer_part_trimmed} * 1000000 + ${fraction_part_trimmed}")
  if(sign LESS 0)
    math(EXPR scaled_value "-1 * ${scaled_value}")
  endif()

  set(${output_variable} "${scaled_value}" PARENT_SCOPE)
endfunction()

function(collect_aggregate_metric_entries json_content names_output values_output)
  string(JSON benchmarks_length ERROR_VARIABLE length_error LENGTH "${json_content}" benchmarks)
  if(length_error)
    message(FATAL_ERROR
      "[bench-compare] Failed to parse benchmark JSON: ${length_error}")
  endif()

  set(entry_names)
  set(entry_values)

  if(benchmarks_length GREATER 0)
    math(EXPR max_index "${benchmarks_length} - 1")
    foreach(index RANGE ${max_index})
      string(JSON run_type ERROR_VARIABLE run_type_error GET
        "${json_content}" benchmarks ${index} run_type)
      if(run_type_error)
        continue()
      endif()
      if(NOT run_type STREQUAL "aggregate")
        continue()
      endif()

      string(JSON aggregate_name ERROR_VARIABLE aggregate_name_error GET
        "${json_content}" benchmarks ${index} aggregate_name)
      if(aggregate_name_error OR NOT aggregate_name STREQUAL "${BENCHMARK_AGGREGATE}")
        continue()
      endif()

      string(JSON benchmark_name ERROR_VARIABLE benchmark_name_error GET
        "${json_content}" benchmarks ${index} name)
      if(benchmark_name_error)
        message(FATAL_ERROR
          "[bench-compare] Aggregate benchmark entry is missing 'name' at index ${index}")
      endif()

      string(JSON metric_value ERROR_VARIABLE metric_error GET
        "${json_content}" benchmarks ${index} ${BENCHMARK_METRIC})
      if(metric_error)
        message(FATAL_ERROR
          "[bench-compare] Aggregate benchmark '${benchmark_name}' is missing metric "
          "'${BENCHMARK_METRIC}'")
      endif()

      list(APPEND entry_names "${benchmark_name}")
      list(APPEND entry_values "${metric_value}")
    endforeach()
  endif()

  set(${names_output} "${entry_names}" PARENT_SCOPE)
  set(${values_output} "${entry_values}" PARENT_SCOPE)
endfunction()

function(find_entry_value entry_names entry_values entry_name output_variable)
  list(LENGTH entry_names name_count)
  if(name_count EQUAL 0)
    set(${output_variable} "" PARENT_SCOPE)
    return()
  endif()

  math(EXPR max_index "${name_count} - 1")
  foreach(index RANGE ${max_index})
    list(GET entry_names ${index} candidate_name)
    if(candidate_name STREQUAL "${entry_name}")
      list(GET entry_values ${index} candidate_value)
      set(${output_variable} "${candidate_value}" PARENT_SCOPE)
      return()
    endif()
  endforeach()

  set(${output_variable} "" PARENT_SCOPE)
endfunction()

require_non_empty(BENCHMARK_BASELINE_PATH)
require_non_empty(BENCHMARK_CANDIDATE_PATH)

if(NOT EXISTS "${BENCHMARK_BASELINE_PATH}")
  message(FATAL_ERROR
    "[bench-compare] Baseline file not found: ${BENCHMARK_BASELINE_PATH}\n"
    "Create a baseline with:\n"
    "  cmake --workflow --preset benchmark-baseline")
endif()

if(NOT EXISTS "${BENCHMARK_CANDIDATE_PATH}")
  message(FATAL_ERROR
    "[bench-compare] Candidate file not found: ${BENCHMARK_CANDIDATE_PATH}\n"
    "Generate benchmark results with:\n"
    "  cmake --workflow --preset benchmark")
endif()

file(READ "${BENCHMARK_BASELINE_PATH}" baseline_json)
file(READ "${BENCHMARK_CANDIDATE_PATH}" candidate_json)

collect_aggregate_metric_entries("${baseline_json}" baseline_names baseline_values)
collect_aggregate_metric_entries("${candidate_json}" candidate_names candidate_values)

list(LENGTH baseline_names baseline_entry_count)
if(baseline_entry_count EQUAL 0)
  message(FATAL_ERROR
    "[bench-compare] No aggregate '${BENCHMARK_AGGREGATE}' entries with metric "
    "'${BENCHMARK_METRIC}' found in baseline: ${BENCHMARK_BASELINE_PATH}")
endif()

list(LENGTH candidate_names candidate_entry_count)
if(candidate_entry_count EQUAL 0)
  message(FATAL_ERROR
    "[bench-compare] No aggregate '${BENCHMARK_AGGREGATE}' entries with metric "
    "'${BENCHMARK_METRIC}' found in candidate: ${BENCHMARK_CANDIDATE_PATH}")
endif()

set(missing_benchmarks)
set(regression_lines)

math(EXPR max_baseline_index "${baseline_entry_count} - 1")
foreach(index RANGE ${max_baseline_index})
  list(GET baseline_names ${index} benchmark_name)
  list(GET baseline_values ${index} baseline_value)

  find_entry_value("${candidate_names}" "${candidate_values}" "${benchmark_name}" candidate_value)
  if("${candidate_value}" STREQUAL "")
    list(APPEND missing_benchmarks "${benchmark_name}")
    continue()
  endif()

  decimal_to_micro_units("${baseline_value}" baseline_scaled)
  decimal_to_micro_units("${candidate_value}" candidate_scaled)

  if(baseline_scaled LESS_EQUAL 0)
    message(FATAL_ERROR
      "[bench-compare] Baseline metric for '${benchmark_name}' must be positive, got: "
      "${baseline_value}")
  endif()

  math(EXPR threshold_scaled
    "${baseline_scaled} + (${baseline_scaled} * ${REGRESSION_THRESHOLD_PERCENT}) / 100")

  if(candidate_scaled GREATER threshold_scaled)
    math(EXPR slowdown_basis_points
      "((${candidate_scaled} - ${baseline_scaled}) * 10000) / ${baseline_scaled}")
    math(EXPR slowdown_whole "${slowdown_basis_points} / 100")
    math(EXPR slowdown_fraction "${slowdown_basis_points} % 100")
    if(slowdown_fraction LESS 10)
      set(slowdown_fraction "0${slowdown_fraction}")
    endif()

    list(APPEND regression_lines
      "  - ${benchmark_name}: baseline=${baseline_value}, candidate=${candidate_value}, slowdown=${slowdown_whole}.${slowdown_fraction}%")
  endif()
endforeach()

list(LENGTH missing_benchmarks missing_count)
if(missing_count GREATER 0)
  string(REPLACE ";" "\n  - " missing_lines "${missing_benchmarks}")
  message(FATAL_ERROR
    "[bench-compare] Candidate is missing benchmark entries present in baseline:\n"
    "  - ${missing_lines}")
endif()

list(LENGTH regression_lines regression_count)
if(regression_count GREATER 0)
  string(REPLACE ";" "\n" regression_report "${regression_lines}")
  message(FATAL_ERROR
    "[bench-compare] Performance regressions exceeded ${REGRESSION_THRESHOLD_PERCENT}% "
    "for '${BENCHMARK_METRIC}' (${BENCHMARK_AGGREGATE}):\n"
    "${regression_report}")
endif()

message(STATUS
  "[bench-compare] PASS: ${baseline_entry_count} benchmark(s) within "
  "${REGRESSION_THRESHOLD_PERCENT}% threshold for ${BENCHMARK_METRIC} (${BENCHMARK_AGGREGATE}).")
