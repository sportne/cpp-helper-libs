# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_path_finding_obj OBJECT)
target_sources(cpphl_path_finding_obj
  PRIVATE
    src/a_star.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_path_finding_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_path_finding_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_path_finding_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

cpphl_apply_static_analysis(cpphl_path_finding_obj)
cpphl_apply_warning_policy(cpphl_path_finding_obj)

# Static library artifact.
add_library(cpphl_path_finding STATIC $<TARGET_OBJECTS:cpphl_path_finding_obj>)
set_target_properties(cpphl_path_finding PROPERTIES OUTPUT_NAME cpphl_path_finding)
add_library(cpphl::path_finding ALIAS cpphl_path_finding)

# Shared library artifact.
add_library(cpphl_path_finding_shared SHARED $<TARGET_OBJECTS:cpphl_path_finding_obj>)
set_target_properties(cpphl_path_finding_shared PROPERTIES OUTPUT_NAME cpphl_path_finding)
add_library(cpphl::path_finding_shared ALIAS cpphl_path_finding_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_path_finding_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(cpphl_path_finding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_path_finding_shared
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_path_finding
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

target_link_libraries(cpphl_path_finding_shared
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
