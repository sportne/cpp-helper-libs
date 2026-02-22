# Define the concrete math library target and a namespaced alias used by consumers.
add_library(cpphl_math)
add_library(cpphl::math ALIAS cpphl_math)

# Source files compiled into the math library.
target_sources(cpphl_math
  PRIVATE
    src/arithmetic.cpp
)

# Public include paths exposed to dependents (build-tree and install-tree variants).
target_include_directories(cpphl_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_math
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

# Apply clang-tidy/cppcheck when static-analysis options are enabled.
cpphl_apply_static_analysis(cpphl_math)
cpphl_apply_warning_policy(cpphl_math)

if(CPPHL_BUILD_TESTS)
  # Module-local unit tests.
  add_subdirectory(tests)
endif()
