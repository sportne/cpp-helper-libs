# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the math module.
add_executable(cpphl_math_tests)
target_sources(cpphl_math_tests
  PRIVATE
    arithmetic_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_math_tests
  PRIVATE
    cpphl::math
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_math_tests)
cpphl_apply_warning_policy(cpphl_math_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_math_tests)
