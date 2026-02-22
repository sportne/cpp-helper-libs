cmake_minimum_required(VERSION 3.25)

project(cpp-helper-libs VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CPPHL_BUILD_TESTS "Build test targets" ON)
option(CPPHL_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(CPPHL_ENABLE_SANITIZERS "Enable sanitizer flags for Clang in Debug-like builds" OFF)
option(CPPHL_ENABLE_COVERAGE "Enable coverage instrumentation for GNU builds" OFF)
option(CPPHL_ENABLE_CLANG_TIDY "Enable clang-tidy static analysis on first-party targets" OFF)
option(CPPHL_ENABLE_CPPCHECK "Enable cppcheck static analysis on first-party targets" OFF)

include(cmake/ProjectOptions.cmake)
include(cmake/Warnings.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Coverage.cmake)
include(cmake/StaticAnalysis.cmake)

cpphl_configure_project_options()
cpphl_configure_warnings()
cpphl_configure_sanitizers()
cpphl_configure_coverage()

if(CPPHL_BUILD_TESTS)
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
      "GoogleTest submodule not found at third_party/googletest. "
      "Run: git submodule update --init --recursive")
  endif()

  add_subdirectory(third_party/googletest EXCLUDE_FROM_ALL)
  enable_testing()
endif()

add_subdirectory(libs)

if(CPPHL_BUILD_TESTS)
  add_subdirectory(tests)
endif()

add_custom_target(format
  COMMAND "${CMAKE_SOURCE_DIR}/scripts/format-cpp.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Formatting C/C++ sources under libs/ and tests/"
  VERBATIM
)

add_custom_target(format-check
  COMMAND "${CMAKE_SOURCE_DIR}/scripts/check-format-cpp.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Checking C/C++ formatting under libs/ and tests/"
  VERBATIM
)

add_custom_target(ci-local
  COMMAND "${CMAKE_SOURCE_DIR}/scripts/run-ci-local.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Running local CI-equivalent checks"
  VERBATIM
)
