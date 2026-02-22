# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the linear_algebra module.
add_executable(cpphl_linear_algebra_tests)
target_sources(cpphl_linear_algebra_tests
  PRIVATE
    matrix3_test.cpp
    matrix_test.cpp
    unit_vector3_test.cpp
    vector3_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_linear_algebra_tests
  PRIVATE
    cpphl::linear_algebra
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_linear_algebra_tests)
cpphl_apply_warning_policy(cpphl_linear_algebra_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_linear_algebra_tests)
