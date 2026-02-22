add_library(cpphl_math)
add_library(cpphl::math ALIAS cpphl_math)

target_sources(cpphl_math
  PRIVATE
    src/arithmetic.cpp
)

target_include_directories(cpphl_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cpphl_math
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

cpphl_apply_static_analysis(cpphl_math)

if(CPPHL_BUILD_TESTS)
  add_subdirectory(tests)
endif()
