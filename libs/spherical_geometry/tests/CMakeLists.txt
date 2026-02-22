# Load helper functions for GoogleTest test discovery.
include(GoogleTest)

# Unit-test executable for the spherical_geometry module.
add_executable(cpphl_spherical_geometry_tests)
target_sources(cpphl_spherical_geometry_tests
  PRIVATE
    coordinate_test.cpp
    spherical_ray_test.cpp
    spherical_curve_test.cpp
    spherical_shape_test.cpp
)

# Link module under test plus gtest main().
target_link_libraries(cpphl_spherical_geometry_tests
  PRIVATE
    cpphl::spherical_geometry
    GTest::gtest_main
)

# Run static analysis on test code when enabled.
cpphl_apply_static_analysis(cpphl_spherical_geometry_tests)
cpphl_apply_warning_policy(cpphl_spherical_geometry_tests)

# Auto-register each TEST(...) case with CTest.
gtest_discover_tests(cpphl_spherical_geometry_tests)
