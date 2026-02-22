# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_spherical_geometry_obj OBJECT)
target_sources(cpphl_spherical_geometry_obj
  PRIVATE
    src/spherical_internal.cpp
    src/coordinate.cpp
    src/intersection.cpp
    src/spherical_ray.cpp
    src/spherical_curve.cpp
    src/minor_arc.cpp
    src/major_arc.cpp
    src/small_arc.cpp
    src/zero_length_curve.cpp
    src/spherical_polygon.cpp
    src/spherical_circle.cpp
    src/spherical_ellipse.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_spherical_geometry_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_spherical_geometry_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_spherical_geometry_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::linear_algebra
    cpphl::quantities
)

cpphl_apply_static_analysis(cpphl_spherical_geometry_obj)
cpphl_apply_warning_policy(cpphl_spherical_geometry_obj)

# Static library artifact.
add_library(cpphl_spherical_geometry STATIC $<TARGET_OBJECTS:cpphl_spherical_geometry_obj>)
set_target_properties(cpphl_spherical_geometry PROPERTIES OUTPUT_NAME cpphl_spherical_geometry)
add_library(cpphl::spherical_geometry ALIAS cpphl_spherical_geometry)

# Shared library artifact.
add_library(cpphl_spherical_geometry_shared SHARED $<TARGET_OBJECTS:cpphl_spherical_geometry_obj>)
set_target_properties(cpphl_spherical_geometry_shared PROPERTIES OUTPUT_NAME cpphl_spherical_geometry)
add_library(cpphl::spherical_geometry_shared ALIAS cpphl_spherical_geometry_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_spherical_geometry_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(cpphl_spherical_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_spherical_geometry_shared
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_spherical_geometry
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::linear_algebra
    cpphl::quantities
)

target_link_libraries(cpphl_spherical_geometry_shared
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
    cpphl::linear_algebra_shared
    cpphl::quantities_shared
)

if(CPPHL_BUILD_TESTS)
  # Module-local unit tests.
  add_subdirectory(tests)
endif()
