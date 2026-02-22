# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_linear_algebra_obj OBJECT)
target_sources(cpphl_linear_algebra_obj
  PRIVATE
    src/vector3.cpp
    src/unit_vector3.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_linear_algebra_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_linear_algebra_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_linear_algebra_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::quantities
)

cpphl_apply_static_analysis(cpphl_linear_algebra_obj)
cpphl_apply_warning_policy(cpphl_linear_algebra_obj)

# Static library artifact.
add_library(cpphl_linear_algebra STATIC $<TARGET_OBJECTS:cpphl_linear_algebra_obj>)
set_target_properties(cpphl_linear_algebra PROPERTIES OUTPUT_NAME cpphl_linear_algebra)
add_library(cpphl::linear_algebra ALIAS cpphl_linear_algebra)

# Shared library artifact.
add_library(cpphl_linear_algebra_shared SHARED $<TARGET_OBJECTS:cpphl_linear_algebra_obj>)
set_target_properties(cpphl_linear_algebra_shared PROPERTIES OUTPUT_NAME cpphl_linear_algebra)
add_library(cpphl::linear_algebra_shared ALIAS cpphl_linear_algebra_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_linear_algebra_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(cpphl_linear_algebra
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_linear_algebra_shared
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_linear_algebra
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::quantities
)

target_link_libraries(cpphl_linear_algebra_shared
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::quantities_shared
)

if(CPPHL_BUILD_TESTS)
  # Module-local unit tests.
  add_subdirectory(tests)
endif()
