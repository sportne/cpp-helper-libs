cmake_minimum_required(VERSION 3.25)

# Define the workspace-level project metadata and supported languages.
project(cpp-helper-libs VERSION 0.1.0 LANGUAGES CXX)

# Set a consistent C++ language level for all targets in this project.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build toggles that can be overridden from presets or command line.
option(CPPHL_BUILD_TESTS "Build test targets" ON)
option(CPPHL_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(CPPHL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors on first-party targets" ON)
option(CPPHL_ENABLE_SANITIZERS "Enable sanitizer flags for Clang in Debug-like builds" OFF)
option(CPPHL_ENABLE_COVERAGE "Enable coverage instrumentation for GNU builds" OFF)
option(CPPHL_ENABLE_CLANG_TIDY "Enable clang-tidy static analysis on first-party targets" OFF)
option(CPPHL_ENABLE_CPPCHECK "Enable cppcheck static analysis on first-party targets" OFF)
option(CPPHL_ENABLE_INCLUDE_WHAT_YOU_USE "Enable include-what-you-use analysis on first-party targets" OFF)
option(CPPHL_ENABLE_CPD "Enable PMD CPD duplicate-code scanning target" ON)
set(CPPHL_CPD_MINIMUM_TOKENS 220 CACHE STRING "Minimum token threshold used by CPD duplicate detection")
option(CPPHL_CPD_FAIL_ON_VIOLATION "Fail CPD target when duplicate blocks are found" OFF)
set(CPPHL_PMD_EXECUTABLE "" CACHE FILEPATH "Optional explicit path to PMD executable (pmd or pmd.bat)")

# Load helper modules that define reusable interface targets/functions.
include(cmake/ProjectOptions.cmake)
include(cmake/Warnings.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Coverage.cmake)
include(cmake/StaticAnalysis.cmake)

# Materialize interface targets (options/warnings/sanitizers/coverage).
cpphl_configure_project_options()
cpphl_configure_warnings()
cpphl_configure_sanitizers()
cpphl_configure_coverage()

if(CPPHL_BUILD_TESTS)
  # Tests depend on GoogleTest, vendored as a git submodule in third_party/.
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
      "GoogleTest submodule not found at third_party/googletest. "
      "Run: git submodule update --init --recursive")
  endif()

  # Bring GoogleTest into the build and register the CTest framework.
  add_subdirectory(third_party/googletest EXCLUDE_FROM_ALL)
  enable_testing()
endif()

# Build all first-party libraries/modules.
add_subdirectory(libs)

if(CPPHL_BUILD_TESTS)
  # Build top-level smoke/integration tests.
  add_subdirectory(tests)
endif()

# Convenience target: apply clang-format to tracked C/C++ files.
add_custom_target(format
  COMMAND "${CMAKE_COMMAND}"
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -DMODE=format
          -P "${CMAKE_SOURCE_DIR}/cmake/RunFormat.cmake"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Formatting C/C++ sources under libs/ and tests/"
  VERBATIM
)

# Convenience target: fail if tracked C/C++ files are not formatted.
add_custom_target(format-check
  COMMAND "${CMAKE_COMMAND}"
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -DMODE=check
          -P "${CMAKE_SOURCE_DIR}/cmake/RunFormat.cmake"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Checking C/C++ formatting under libs/ and tests/"
  VERBATIM
)

# Convenience target: run PMD CPD duplicate-code scan over first-party libs.
add_custom_target(cpd
  COMMAND "${CMAKE_COMMAND}"
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -DCPPHL_ENABLE_CPD=${CPPHL_ENABLE_CPD}
          -DCPPHL_CPD_MINIMUM_TOKENS=${CPPHL_CPD_MINIMUM_TOKENS}
          -DCPPHL_CPD_FAIL_ON_VIOLATION=${CPPHL_CPD_FAIL_ON_VIOLATION}
          -DCPPHL_PMD_EXECUTABLE=${CPPHL_PMD_EXECUTABLE}
          -P "${CMAKE_SOURCE_DIR}/cmake/RunCpd.cmake"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Running PMD CPD duplicate-code scan"
  VERBATIM
)

# Convenience target: run the same quality gates as CI in sequence.
add_custom_target(ci-local
  COMMAND "${CMAKE_COMMAND}"
          -DREPO_ROOT=${CMAKE_SOURCE_DIR}
          -P "${CMAKE_SOURCE_DIR}/cmake/RunCiLocal.cmake"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Running local CI-equivalent checks"
  VERBATIM
)
