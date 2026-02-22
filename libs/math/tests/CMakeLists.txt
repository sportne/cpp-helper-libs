include(GoogleTest)

add_executable(cpphl_math_tests)
target_sources(cpphl_math_tests
  PRIVATE
    arithmetic_test.cpp
)

target_link_libraries(cpphl_math_tests
  PRIVATE
    cpphl::math
    GTest::gtest_main
)

gtest_discover_tests(cpphl_math_tests)
