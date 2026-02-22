# Compile sources once and reuse objects for both static and shared libraries.
add_library(cpphl_quantities_obj OBJECT)
target_sources(cpphl_quantities_obj
  PRIVATE
    src/length.cpp
    src/time.cpp
    src/mass.cpp
    src/force.cpp
    src/speed.cpp
    src/acceleration.cpp
    src/angle.cpp
)

# Shared library consumes object files, so force PIC for portable builds.
set_target_properties(cpphl_quantities_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cpphl_quantities_obj
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpphl_quantities_obj
  PRIVATE
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

cpphl_apply_static_analysis(cpphl_quantities_obj)
cpphl_apply_warning_policy(cpphl_quantities_obj)

# Static library artifact.
add_library(cpphl_quantities STATIC $<TARGET_OBJECTS:cpphl_quantities_obj>)
set_target_properties(cpphl_quantities PROPERTIES OUTPUT_NAME cpphl_quantities)
add_library(cpphl::quantities ALIAS cpphl_quantities)

# Shared library artifact.
add_library(cpphl_quantities_shared SHARED $<TARGET_OBJECTS:cpphl_quantities_obj>)
set_target_properties(cpphl_quantities_shared PROPERTIES OUTPUT_NAME cpphl_quantities)
add_library(cpphl::quantities_shared ALIAS cpphl_quantities_shared)

# Improve Windows compatibility for simple C-style exports.
if(MSVC)
  set_target_properties(cpphl_quantities_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(cpphl_quantities
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(cpphl_quantities_shared
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link shared project settings via interface targets rather than global flags.
target_link_libraries(cpphl_quantities
  PUBLIC
    cpphl_project_options
    cpphl_warnings
    cpphl_sanitizers
    cpphl_coverage
)

target_link_libraries(cpphl_quantities_shared
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
