if(NOT DEFINED METRIC)
  set(METRIC "cpu_time")
endif()

if(NOT DEFINED AGGREGATE)
  set(AGGREGATE "median")
endif()

if(NOT DEFINED TOP_N)
  set(TOP_N 10)
endif()

function(require_non_empty variable_name)
  if(NOT DEFINED ${variable_name} OR "${${variable_name}}" STREQUAL "")
    message(FATAL_ERROR "[bench-hotspots] Required variable '${variable_name}' is not set")
  endif()
endfunction()

function(decimal_to_micro_units decimal_value output_variable)
  # Convert decimal strings to integer micro-units so sorting is deterministic.
  if(NOT "${decimal_value}" MATCHES "^-?[0-9]+(\\.[0-9]+)?$")
    message(FATAL_ERROR
      "[bench-hotspots] Unsupported numeric value '${decimal_value}'. "
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

require_non_empty(BENCHMARK_JSON_PATH)
require_non_empty(OUTPUT_SUMMARY_PATH)

if(NOT EXISTS "${BENCHMARK_JSON_PATH}")
  message(FATAL_ERROR "[bench-hotspots] Benchmark JSON not found: ${BENCHMARK_JSON_PATH}")
endif()

file(READ "${BENCHMARK_JSON_PATH}" benchmark_json)
string(JSON benchmark_count ERROR_VARIABLE benchmark_count_error LENGTH
  "${benchmark_json}" benchmarks)
if(benchmark_count_error)
  message(FATAL_ERROR "[bench-hotspots] Failed to parse benchmark JSON: ${benchmark_count_error}")
endif()

set(sorted_rows)
set(total_rows 0)
if(benchmark_count GREATER 0)
  math(EXPR max_index "${benchmark_count} - 1")
  foreach(index RANGE ${max_index})
    string(JSON run_type ERROR_VARIABLE run_type_error GET
      "${benchmark_json}" benchmarks ${index} run_type)
    if(run_type_error OR NOT run_type STREQUAL "aggregate")
      continue()
    endif()

    string(JSON aggregate_name ERROR_VARIABLE aggregate_error GET
      "${benchmark_json}" benchmarks ${index} aggregate_name)
    if(aggregate_error OR NOT aggregate_name STREQUAL "${AGGREGATE}")
      continue()
    endif()

    string(JSON benchmark_name ERROR_VARIABLE name_error GET
      "${benchmark_json}" benchmarks ${index} name)
    if(name_error)
      continue()
    endif()

    string(JSON metric_value ERROR_VARIABLE metric_error GET
      "${benchmark_json}" benchmarks ${index} ${METRIC})
    if(metric_error)
      continue()
    endif()

    string(JSON metric_unit ERROR_VARIABLE unit_error GET
      "${benchmark_json}" benchmarks ${index} time_unit)
    if(unit_error)
      set(metric_unit "")
    endif()

    decimal_to_micro_units("${metric_value}" metric_scaled)
    if(metric_scaled LESS_EQUAL 0)
      continue()
    endif()

    # Lexicographic sort key: fixed-width scaled metric first (descending).
    set(metric_scaled_padded "${metric_scaled}")
    string(LENGTH "${metric_scaled_padded}" padded_length)
    while(padded_length LESS 20)
      set(metric_scaled_padded "0${metric_scaled_padded}")
      math(EXPR padded_length "${padded_length} + 1")
    endwhile()

    list(APPEND sorted_rows
      "${metric_scaled_padded}|${benchmark_name}|${metric_value}|${metric_unit}")
    math(EXPR total_rows "${total_rows} + 1")
  endforeach()
endif()

if(total_rows EQUAL 0)
  message(FATAL_ERROR
    "[bench-hotspots] No aggregate '${AGGREGATE}' entries found for metric '${METRIC}' "
    "in ${BENCHMARK_JSON_PATH}")
endif()

list(SORT sorted_rows ORDER DESCENDING)

set(top_limit "${TOP_N}")
if(top_limit GREATER total_rows)
  set(top_limit "${total_rows}")
endif()
math(EXPR top_last_index "${top_limit} - 1")

get_filename_component(summary_dir "${OUTPUT_SUMMARY_PATH}" DIRECTORY)
file(MAKE_DIRECTORY "${summary_dir}")

set(summary_body "# Spherical Geometry Benchmark Hotspots\n\n")
string(APPEND summary_body
  "Generated from `${BENCHMARK_JSON_PATH}` using aggregate `${AGGREGATE}` for metric "
  "`${METRIC}`.\n\n")
string(APPEND summary_body "| Rank | Benchmark | ${AGGREGATE} ${METRIC} |\n")
string(APPEND summary_body "|---:|---|---:|\n")

set(top_benchmark_name "")
set(top_metric_value "")
foreach(index RANGE ${top_last_index})
  list(GET sorted_rows ${index} row)
  string(REPLACE "|" ";" row_parts "${row}")
  list(GET row_parts 1 benchmark_name)
  list(GET row_parts 2 metric_value)
  list(GET row_parts 3 metric_unit)

  if(index EQUAL 0)
    set(top_benchmark_name "${benchmark_name}")
    set(top_metric_value "${metric_value}")
  endif()

  math(EXPR rank "${index} + 1")
  if(metric_unit STREQUAL "")
    string(APPEND summary_body "| ${rank} | `${benchmark_name}` | ${metric_value} |\n")
  else()
    string(APPEND summary_body
      "| ${rank} | `${benchmark_name}` | ${metric_value} ${metric_unit} |\n")
  endif()
endforeach()

string(APPEND summary_body "\n"
  "Top hotspot candidate: `${top_benchmark_name}` (${top_metric_value}).\n")

file(WRITE "${OUTPUT_SUMMARY_PATH}" "${summary_body}")
message(STATUS "[bench-hotspots] wrote summary: ${OUTPUT_SUMMARY_PATH}")
