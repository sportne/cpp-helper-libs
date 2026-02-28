# Spherical-geometry benchmark executable.
add_executable(cpphl_spherical_geometry_benchmarks)
target_sources(cpphl_spherical_geometry_benchmarks
  PRIVATE
    spherical_geometry_benchmark.cpp
)

target_link_libraries(cpphl_spherical_geometry_benchmarks
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::spherical_geometry
    benchmark::benchmark_main
)

cpphl_apply_static_analysis(cpphl_spherical_geometry_benchmarks)
cpphl_apply_warning_policy(cpphl_spherical_geometry_benchmarks)

set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_DIR
    "${CMAKE_BINARY_DIR}/benchmarks/spherical_geometry")
set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_LATEST_JSON
    "${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_DIR}/latest.json")
set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_BASELINE_JSON
    "${CMAKE_SOURCE_DIR}/docs/benchmarks/spherical_geometry-baseline.local.json")

set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_COMMON_ARGS
    --benchmark_repetitions=7
    --benchmark_min_time=0.20s
    --benchmark_report_aggregates_only=true
    --benchmark_display_aggregates_only=true
)

# Run benchmarks and write machine-readable results into the build tree.
add_custom_target(bench-spherical-geometry
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_DIR}"
  COMMAND "$<TARGET_FILE:cpphl_spherical_geometry_benchmarks>"
          ${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_COMMON_ARGS}
          --benchmark_out=${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_LATEST_JSON}
          --benchmark_out_format=json
  DEPENDS cpphl_spherical_geometry_benchmarks
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Running spherical-geometry benchmarks (console + JSON output)"
  VERBATIM
)

# Record a local baseline file for this machine.
add_custom_target(bench-baseline-spherical-geometry
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_SOURCE_DIR}/docs/benchmarks"
  COMMAND "$<TARGET_FILE:cpphl_spherical_geometry_benchmarks>"
          ${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_COMMON_ARGS}
          --benchmark_out=${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_BASELINE_JSON}
          --benchmark_out_format=json
  DEPENDS cpphl_spherical_geometry_benchmarks
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Recording local spherical-geometry benchmark baseline"
  VERBATIM
)

# Compare the latest run against the local baseline and fail on >10% regressions.
add_custom_target(bench-compare-spherical-geometry
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_BASELINE_PATH=${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_BASELINE_JSON}
          -DBENCHMARK_CANDIDATE_PATH=${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_LATEST_JSON}
          -DBENCHMARK_METRIC=cpu_time
          -DBENCHMARK_AGGREGATE=median
          -DREGRESSION_THRESHOLD_PERCENT=10
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -P "${CMAKE_SOURCE_DIR}/cmake/CompareBenchmarks.cmake"
  DEPENDS bench-spherical-geometry
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Comparing spherical-geometry benchmark results against local baseline"
  VERBATIM
)
