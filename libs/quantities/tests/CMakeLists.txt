# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the quantities module.
add_executable(cpphl_quantities_tests)
target_sources(cpphl_quantities_tests
  PRIVATE
    quantities_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_quantities_tests
  PRIVATE
    cpphl::quantities
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_quantities_tests)
cpphl_apply_warning_policy(cpphl_quantities_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_quantities_tests)
