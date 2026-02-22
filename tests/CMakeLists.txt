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
    GTest::gtest_main
)

cpphl_apply_static_analysis(cpphl_smoke_shared_tests)
cpphl_apply_warning_policy(cpphl_smoke_shared_tests)

# Discover and register GoogleTest cases with CTest.
include(GoogleTest)
gtest_discover_tests(cpphl_smoke_tests)
gtest_discover_tests(cpphl_smoke_shared_tests)
