if(NOT DEFINED REPO_ROOT)
  # Default to repository root (one directory up from this script file).
  get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

if(NOT DEFINED CPPHL_ENABLE_CPD)
  set(CPPHL_ENABLE_CPD ON)
endif()

if(NOT CPPHL_ENABLE_CPD)
  message(STATUS "[cpd] disabled via CPPHL_ENABLE_CPD=OFF")
  return()
endif()

if(NOT DEFINED CPPHL_CPD_MINIMUM_TOKENS)
  set(CPPHL_CPD_MINIMUM_TOKENS 220)
endif()

if(NOT DEFINED CPPHL_CPD_FAIL_ON_VIOLATION)
  set(CPPHL_CPD_FAIL_ON_VIOLATION OFF)
endif()

if(NOT DEFINED CPPHL_PMD_EXECUTABLE)
  set(CPPHL_PMD_EXECUTABLE "")
endif()

set(CPPHL_PMD_RESOLVED_EXECUTABLE "")

# 1) Explicit PMD path override.
if(NOT CPPHL_PMD_EXECUTABLE STREQUAL "")
  if(EXISTS "${CPPHL_PMD_EXECUTABLE}")
    set(CPPHL_PMD_RESOLVED_EXECUTABLE "${CPPHL_PMD_EXECUTABLE}")
  else()
    message(FATAL_ERROR "[cpd] CPPHL_PMD_EXECUTABLE does not exist: ${CPPHL_PMD_EXECUTABLE}")
  endif()
endif()

# 2) PMD from PATH.
if(CPPHL_PMD_RESOLVED_EXECUTABLE STREQUAL "")
  find_program(CPPHL_PMD_FROM_PATH NAMES pmd pmd.bat)
  if(CPPHL_PMD_FROM_PATH)
    set(CPPHL_PMD_RESOLVED_EXECUTABLE "${CPPHL_PMD_FROM_PATH}")
  endif()
endif()

# 3) Fallback under third_party/pmd/*/bin/.
if(CPPHL_PMD_RESOLVED_EXECUTABLE STREQUAL "")
  file(GLOB CPPHL_PMD_FALLBACK_CANDIDATES
    "${REPO_ROOT}/third_party/pmd/*/bin/pmd"
    "${REPO_ROOT}/third_party/pmd/*/bin/pmd.bat"
    "${REPO_ROOT}/third_party/pmd/*/bin/pmd.cmd"
  )
  list(LENGTH CPPHL_PMD_FALLBACK_CANDIDATES CPPHL_PMD_FALLBACK_COUNT)
  if(CPPHL_PMD_FALLBACK_COUNT GREATER 0)
    list(SORT CPPHL_PMD_FALLBACK_CANDIDATES)
    list(GET CPPHL_PMD_FALLBACK_CANDIDATES 0 CPPHL_PMD_RESOLVED_EXECUTABLE)
  endif()
endif()

if(CPPHL_PMD_RESOLVED_EXECUTABLE STREQUAL "")
  message(STATUS "[cpd] PMD not found; skipping CPD scan")
  return()
endif()

find_program(GIT_EXECUTABLE NAMES git REQUIRED)

execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${REPO_ROOT}" ls-files libs
  OUTPUT_VARIABLE tracked_files_raw
  RESULT_VARIABLE tracked_files_result
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT tracked_files_result EQUAL 0)
  message(FATAL_ERROR "[cpd] failed to enumerate tracked files with git")
endif()

if(tracked_files_raw STREQUAL "")
  message(STATUS "[cpd] no tracked files found under libs/")
  return()
endif()

string(REPLACE "\n" ";" tracked_files "${tracked_files_raw}")

set(cpd_files)
foreach(rel_path IN LISTS tracked_files)
  if(NOT rel_path MATCHES "^libs/.*/(src|include)/")
    continue()
  endif()
  if(rel_path MATCHES "/tests/")
    continue()
  endif()
  if(NOT rel_path MATCHES "\\.(h|hh|hpp|c|cc|cpp|cxx)$")
    continue()
  endif()

  set(abs_path "${REPO_ROOT}/${rel_path}")
  if(EXISTS "${abs_path}")
    list(APPEND cpd_files "${abs_path}")
  endif()
endforeach()

list(LENGTH cpd_files cpd_file_count)
if(cpd_file_count EQUAL 0)
  message(STATUS "[cpd] no matching source/header files to scan")
  return()
endif()

set(cpd_output_dir "${REPO_ROOT}/build/cpd")
set(cpd_file_list_path "${cpd_output_dir}/cpd-file-list.txt")
set(cpd_report_path "${cpd_output_dir}/cpd-report.md")
file(MAKE_DIRECTORY "${cpd_output_dir}")

string(REPLACE ";" "\n" cpd_file_list_content "${cpd_files}")
file(WRITE "${cpd_file_list_path}" "${cpd_file_list_content}\n")

set(cpd_command
  "${CPPHL_PMD_RESOLVED_EXECUTABLE}" cpd
  --language cpp
  --minimum-tokens "${CPPHL_CPD_MINIMUM_TOKENS}"
  --ignore-identifiers
  --ignore-literals
  --file-list "${cpd_file_list_path}"
  --format markdown
  --report-file "${cpd_report_path}"
  --relativize-paths-with "${REPO_ROOT}"
)

if(NOT CPPHL_CPD_FAIL_ON_VIOLATION)
  list(APPEND cpd_command --no-fail-on-violation)
endif()

message(STATUS "[cpd] using PMD executable: ${CPPHL_PMD_RESOLVED_EXECUTABLE}")
message(STATUS "[cpd] scanning ${cpd_file_count} files (minimum tokens: ${CPPHL_CPD_MINIMUM_TOKENS})")

execute_process(
  COMMAND ${cpd_command}
  WORKING_DIRECTORY "${REPO_ROOT}"
  RESULT_VARIABLE cpd_result
)

if(cpd_result EQUAL 0)
  message(STATUS "[cpd] report generated: ${cpd_report_path}")
  return()
endif()

if(cpd_result EQUAL 4)
  if(CPPHL_CPD_FAIL_ON_VIOLATION)
    message(FATAL_ERROR "[cpd] duplications found (report: ${cpd_report_path})")
  endif()
  message(STATUS "[cpd] duplications found (report-only mode): ${cpd_report_path}")
  return()
endif()

message(FATAL_ERROR "[cpd] PMD CPD failed with exit code ${cpd_result}")
