# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_dubins_path_finding_obj OBJECT)
target_sources(cpphl_dubins_path_finding_obj
  PRIVATE
    src/spherical_dubins_planner.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_dubins_path_finding_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_dubins_path_finding_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_dubins_path_finding_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::path_finding
    cpphl::spherical_geometry
    cpphl::linear_algebra
    cpphl::quantities
)

cpphl_apply_static_analysis(cpphl_dubins_path_finding_obj)
cpphl_apply_warning_policy(cpphl_dubins_path_finding_obj)

# Static library artifact.
add_library(cpphl_dubins_path_finding STATIC $<TARGET_OBJECTS:cpphl_dubins_path_finding_obj>)
set_target_properties(cpphl_dubins_path_finding PROPERTIES OUTPUT_NAME cpphl_dubins_path_finding)
add_library(cpphl::dubins_path_finding ALIAS cpphl_dubins_path_finding)

# Shared library artifact.
add_library(cpphl_dubins_path_finding_shared SHARED $<TARGET_OBJECTS:cpphl_dubins_path_finding_obj>)
set_target_properties(cpphl_dubins_path_finding_shared PROPERTIES OUTPUT_NAME cpphl_dubins_path_finding)
add_library(cpphl::dubins_path_finding_shared ALIAS cpphl_dubins_path_finding_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_dubins_path_finding_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(cpphl_dubins_path_finding
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_dubins_path_finding_shared
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_dubins_path_finding
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::path_finding
    cpphl::spherical_geometry
    cpphl::linear_algebra
    cpphl::quantities
)

target_link_libraries(cpphl_dubins_path_finding_shared
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::path_finding_shared
    cpphl::spherical_geometry_shared
    cpphl::linear_algebra_shared
    cpphl::quantities_shared
)

if(CPPHL_BUILD_TESTS)
  # Module-local unit tests.
  add_subdirectory(tests)
endif()
