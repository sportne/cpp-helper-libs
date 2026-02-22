add_executable(cpphl_smoke_tests)
target_sources(cpphl_smoke_tests
  PRIVATE
    smoke/project_smoke_test.cpp
)

target_link_libraries(cpphl_smoke_tests
  PRIVATE
    cpphl::math
    GTest::gtest_main
)

cpphl_apply_static_analysis(cpphl_smoke_tests)

include(GoogleTest)
gtest_discover_tests(cpphl_smoke_tests)
