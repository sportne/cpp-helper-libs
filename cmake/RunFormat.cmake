if(NOT DEFINED REPO_ROOT)
  # Default to repository root (one directory up from this script file).
  get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

if(NOT DEFINED MODE)
  # Default mode is format validation.
  set(MODE "check")
endif()

if(NOT MODE STREQUAL "check" AND NOT MODE STREQUAL "format")
  message(FATAL_ERROR "MODE must be either 'check' or 'format'.")
endif()

# Resolve required tools up front with clear configure-time failures.
find_program(GIT_EXECUTABLE NAMES git REQUIRED)
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format REQUIRED)

# Enumerate tracked files only, so generated/untracked files are ignored.
execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${REPO_ROOT}" ls-files libs tests
  OUTPUT_VARIABLE tracked_files_raw
  RESULT_VARIABLE tracked_files_result
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT tracked_files_result EQUAL 0)
  message(FATAL_ERROR "Failed to enumerate tracked files with git.")
endif()

if(tracked_files_raw STREQUAL "")
  message(STATUS "No tracked files found under libs/ or tests/.")
  return()
endif()

string(REPLACE "\n" ";" tracked_files "${tracked_files_raw}")
set(cpp_files)
set(skipped_missing_files 0)
foreach(rel_path IN LISTS tracked_files)
  # Restrict formatting to C/C++ source/header extensions.
  if(rel_path MATCHES "\\.(h|hh|hpp|c|cc|cpp|cxx)$")
    set(abs_path "${REPO_ROOT}/${rel_path}")
    # Handle tracked files that are deleted locally but not yet staged via `git rm`.
    if(EXISTS "${abs_path}")
      list(APPEND cpp_files "${abs_path}")
    else()
      math(EXPR skipped_missing_files "${skipped_missing_files} + 1")
    endif()
  endif()
endforeach()

if(cpp_files STREQUAL "")
  message(STATUS "No C/C++ files found under libs/ or tests/.")
  return()
endif()

foreach(file_path IN LISTS cpp_files)
  if(MODE STREQUAL "format")
    # In-place formatting mode.
    execute_process(
      COMMAND "${CLANG_FORMAT_EXECUTABLE}" -i "${file_path}"
      RESULT_VARIABLE format_result
    )
    if(NOT format_result EQUAL 0)
      message(FATAL_ERROR "clang-format failed for ${file_path}")
    endif()
  else()
    # Validation mode used by CI/format-check target.
    execute_process(
      COMMAND "${CLANG_FORMAT_EXECUTABLE}" --dry-run --Werror "${file_path}"
      RESULT_VARIABLE check_result
    )
    if(NOT check_result EQUAL 0)
      message(FATAL_ERROR "Formatting check failed for ${file_path}")
    endif()
  endif()
endforeach()

list(LENGTH cpp_files file_count)
if(MODE STREQUAL "format")
  message(STATUS "Formatted ${file_count} C/C++ files (skipped ${skipped_missing_files} missing).")
else()
  message(STATUS
          "Formatting check passed for ${file_count} C/C++ files (skipped ${skipped_missing_files} missing).")
endif()
