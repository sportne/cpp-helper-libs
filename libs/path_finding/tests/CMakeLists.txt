# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the path_finding module.
add_executable(cpphl_path_finding_tests)
target_sources(cpphl_path_finding_tests
  PRIVATE
    a_star_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_path_finding_tests
  PRIVATE
    cpphl::path_finding
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_path_finding_tests)
cpphl_apply_warning_policy(cpphl_path_finding_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_path_finding_tests)
