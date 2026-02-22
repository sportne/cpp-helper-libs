# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_math_obj OBJECT)
target_sources(cpphl_math_obj
  PRIVATE
    src/arithmetic.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_math_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_math_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_math_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

cpphl_apply_static_analysis(cpphl_math_obj)
cpphl_apply_warning_policy(cpphl_math_obj)

# Static library artifact.
add_library(cpphl_math STATIC $<TARGET_OBJECTS:cpphl_math_obj>)
set_target_properties(cpphl_math PROPERTIES OUTPUT_NAME cpphl_math)
add_library(cpphl::math ALIAS cpphl_math)

# Shared library artifact.
add_library(cpphl_math_shared SHARED $<TARGET_OBJECTS:cpphl_math_obj>)
set_target_properties(cpphl_math_shared PROPERTIES OUTPUT_NAME cpphl_math)
add_library(cpphl::math_shared ALIAS cpphl_math_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_math_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Public include paths exposed to dependents (build-tree and install-tree variants).
target_include_directories(cpphl_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_math_shared
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

target_link_libraries(cpphl_math_shared
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

if(CPPHL_BUILD_TESTS)
  # Module-local unit tests.
  add_subdirectory(tests)
endif()
