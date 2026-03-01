# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the dubins_path_finding module.
add_executable(cpphl_dubins_path_finding_tests)
target_sources(cpphl_dubins_path_finding_tests
  PRIVATE
    spherical_dubins_planner_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_dubins_path_finding_tests
  PRIVATE
    cpphl::dubins_path_finding
    cpphl::spherical_geometry
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_dubins_path_finding_tests)
cpphl_apply_warning_policy(cpphl_dubins_path_finding_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_dubins_path_finding_tests)
