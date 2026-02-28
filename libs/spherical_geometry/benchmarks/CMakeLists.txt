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
set(CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR
    "${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_DIR}/analysis")

set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_LATEST_JSON
    "${CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_DIR}/latest.json")
set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_BASELINE_JSON
    "${CMAKE_SOURCE_DIR}/docs/benchmarks/spherical_geometry-baseline.local.json")

set(CPPHL_SPHERICAL_GEOMETRY_HOTSPOTS_JSON
    "${CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR}/hotspots.json")
set(CPPHL_SPHERICAL_GEOMETRY_HOTSPOTS_SUMMARY
    "${CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR}/hotspots-summary.md")

set(CPPHL_SPHERICAL_GEOMETRY_PERFSTAT_DIR
    "${CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR}/perfstat")
set(CPPHL_SPHERICAL_GEOMETRY_PERFSTAT_CSV
    "${CPPHL_SPHERICAL_GEOMETRY_PERFSTAT_DIR}/perfstat.csv")

set(CPPHL_SPHERICAL_GEOMETRY_PROFILE_DIR
    "${CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR}/profile")
set(CPPHL_SPHERICAL_GEOMETRY_PROFILE_DATA
    "${CPPHL_SPHERICAL_GEOMETRY_PROFILE_DIR}/perf.data")
set(CPPHL_SPHERICAL_GEOMETRY_PROFILE_REPORT
    "${CPPHL_SPHERICAL_GEOMETRY_PROFILE_DIR}/perf-report.txt")

set(CPPHL_SPHERICAL_GEOMETRY_BENCHMARK_COMMON_ARGS
    --benchmark_repetitions=7
    --benchmark_min_time=0.20s
    --benchmark_report_aggregates_only=true
    --benchmark_display_aggregates_only=true
)

set(CPPHL_SPHERICAL_GEOMETRY_HOTSPOT_ARGS
    --benchmark_repetitions=12
    --benchmark_min_time=0.35s
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

# Generate a slower/stable run plus top-hotspot summary report.
add_custom_target(bench-hotspots-spherical-geometry
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CPPHL_SPHERICAL_GEOMETRY_ANALYSIS_DIR}"
  COMMAND "$<TARGET_FILE:cpphl_spherical_geometry_benchmarks>"
          ${CPPHL_SPHERICAL_GEOMETRY_HOTSPOT_ARGS}
          --benchmark_out=${CPPHL_SPHERICAL_GEOMETRY_HOTSPOTS_JSON}
          --benchmark_out_format=json
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_JSON_PATH=${CPPHL_SPHERICAL_GEOMETRY_HOTSPOTS_JSON}
          -DOUTPUT_SUMMARY_PATH=${CPPHL_SPHERICAL_GEOMETRY_HOTSPOTS_SUMMARY}
          -DMETRIC=cpu_time
          -DAGGREGATE=median
          -DTOP_N=12
          -P "${CMAKE_SOURCE_DIR}/cmake/SummarizeBenchmarkHotspots.cmake"
  DEPENDS cpphl_spherical_geometry_benchmarks
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Running hotspot-oriented spherical-geometry benchmarks and summary"
  VERBATIM
)

# Collect per-benchmark Linux perf stat counters into CSV and raw logs.
add_custom_target(bench-perfstat-spherical-geometry
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_EXECUTABLE=$<TARGET_FILE:cpphl_spherical_geometry_benchmarks>
          -DOUTPUT_DIR=${CPPHL_SPHERICAL_GEOMETRY_PERFSTAT_DIR}
          -DCSV_OUTPUT_PATH=${CPPHL_SPHERICAL_GEOMETRY_PERFSTAT_CSV}
          -DPERF_EVENTS=cycles,instructions,cache-misses,branch-misses
          -P "${CMAKE_SOURCE_DIR}/cmake/RunPerfStatBenchmarks.cmake"
  DEPENDS cpphl_spherical_geometry_benchmarks
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Collecting perf stat counters for spherical-geometry benchmarks"
  VERBATIM
)

# Record sampled call stacks and produce a text hotspot report.
add_custom_target(bench-profile-spherical-geometry
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_EXECUTABLE=$<TARGET_FILE:cpphl_spherical_geometry_benchmarks>
          -DOUTPUT_DIR=${CPPHL_SPHERICAL_GEOMETRY_PROFILE_DIR}
          -DPERF_DATA_PATH=${CPPHL_SPHERICAL_GEOMETRY_PROFILE_DATA}
          -DPERF_REPORT_PATH=${CPPHL_SPHERICAL_GEOMETRY_PROFILE_REPORT}
          "-DPROFILE_BENCHMARK_FILTER=^BM_MinorArcIntersections(/.*)?$"
          -P "${CMAKE_SOURCE_DIR}/cmake/RunPerfProfile.cmake"
  DEPENDS cpphl_spherical_geometry_benchmarks
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Sampling spherical-geometry hotspots with perf record/report"
  VERBATIM
)
