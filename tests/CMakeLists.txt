# Top-level smoke/integration test executable.
add_executable(cpphl_smoke_tests)
target_sources(cpphl_smoke_tests
  PRIVATE
    smoke/project_smoke_test.cpp
)

# Smoke tests link against public module APIs plus gtest main().
target_link_libraries(cpphl_smoke_tests
  PRIVATE
    cpphl::math
    cpphl::quantities
    cpphl::linear_algebra
    cpphl::spherical_geometry
    cpphl::path_finding
    cpphl::dubins_path_finding
    GTest::gtest_main
)

# Run static analysis on smoke tests when enabled.
cpphl_apply_static_analysis(cpphl_smoke_tests)
cpphl_apply_warning_policy(cpphl_smoke_tests)

# Shared-library smoke/integration test executable.
add_executable(cpphl_smoke_shared_tests)
target_sources(cpphl_smoke_shared_tests
  PRIVATE
    smoke/project_smoke_shared_test.cpp
)

target_link_libraries(cpphl_smoke_shared_tests
  PRIVATE
    cpphl::math_shared
    cpphl::quantities_shared
    cpphl::linear_algebra_shared
    cpphl::spherical_geometry_shared
    cpphl::path_finding_shared
    cpphl::dubins_path_finding_shared
    GTest::gtest_main
)

cpphl_apply_static_analysis(cpphl_smoke_shared_tests)
cpphl_apply_warning_policy(cpphl_smoke_shared_tests)

# Discover and register GoogleTest cases with CTest.
include(GoogleTest)
gtest_discover_tests(cpphl_smoke_tests)
gtest_discover_tests(cpphl_smoke_shared_tests)

set(cpphl_benchmark_compare_script "${CMAKE_SOURCE_DIR}/cmake/CompareBenchmarks.cmake")
set(cpphl_benchmark_hotspot_summary_script
    "${CMAKE_SOURCE_DIR}/cmake/SummarizeBenchmarkHotspots.cmake")
set(cpphl_benchmark_fixture_dir "${CMAKE_SOURCE_DIR}/tests/data/benchmarks")

add_test(
  NAME benchmark-compare-pass
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_BASELINE_PATH=${cpphl_benchmark_fixture_dir}/baseline.json
          -DBENCHMARK_CANDIDATE_PATH=${cpphl_benchmark_fixture_dir}/candidate-pass.json
          -DBENCHMARK_METRIC=cpu_time
          -DBENCHMARK_AGGREGATE=median
          -DREGRESSION_THRESHOLD_PERCENT=10
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -P "${cpphl_benchmark_compare_script}"
)

add_test(
  NAME benchmark-compare-fail
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_BASELINE_PATH=${cpphl_benchmark_fixture_dir}/baseline.json
          -DBENCHMARK_CANDIDATE_PATH=${cpphl_benchmark_fixture_dir}/candidate-fail.json
          -DBENCHMARK_METRIC=cpu_time
          -DBENCHMARK_AGGREGATE=median
          -DREGRESSION_THRESHOLD_PERCENT=10
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -P "${cpphl_benchmark_compare_script}"
)
set_tests_properties(benchmark-compare-fail PROPERTIES WILL_FAIL TRUE)

add_test(
  NAME benchmark-compare-missing-baseline
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_BASELINE_PATH=${cpphl_benchmark_fixture_dir}/missing-baseline.json
          -DBENCHMARK_CANDIDATE_PATH=${cpphl_benchmark_fixture_dir}/candidate-pass.json
          -DBENCHMARK_METRIC=cpu_time
          -DBENCHMARK_AGGREGATE=median
          -DREGRESSION_THRESHOLD_PERCENT=10
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -P "${cpphl_benchmark_compare_script}"
)
set_tests_properties(benchmark-compare-missing-baseline PROPERTIES WILL_FAIL TRUE)

add_test(
  NAME benchmark-hotspots-summary
  COMMAND "${CMAKE_COMMAND}"
          -DBENCHMARK_JSON_PATH=${cpphl_benchmark_fixture_dir}/baseline.json
          -DOUTPUT_SUMMARY_PATH=${CMAKE_BINARY_DIR}/benchmarks/hotspots-summary-test.md
          -DMETRIC=cpu_time
          -DAGGREGATE=median
          -DTOP_N=2
          -P "${cpphl_benchmark_hotspot_summary_script}"
)
