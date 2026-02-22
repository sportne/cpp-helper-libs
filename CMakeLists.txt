cmake_minimum_required(VERSION 3.25)

project(cpp-helper-libs VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CPPHL_BUILD_TESTS "Build test targets" ON)
option(CPPHL_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(CPPHL_ENABLE_SANITIZERS "Enable sanitizer flags for Clang in Debug-like builds" OFF)

include(cmake/ProjectOptions.cmake)
include(cmake/Warnings.cmake)
include(cmake/Sanitizers.cmake)

cpphl_configure_project_options()
cpphl_configure_warnings()
cpphl_configure_sanitizers()

if(CPPHL_BUILD_TESTS)
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
      "GoogleTest submodule not found at third_party/googletest. "
      "Run: git submodule update --init --recursive")
  endif()

  add_subdirectory(third_party/googletest EXCLUDE_FROM_ALL)
endif()

add_subdirectory(libs)

if(CPPHL_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
