function(cpphl_apply_static_analysis target_name)
  # Attach clang-tidy per-target (instead of globally) so third_party code is
  # not analyzed unintentionally.
  if(CPPHL_ENABLE_CLANG_TIDY)
    find_program(CPPHL_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(clang_tidy_command
      "${CPPHL_CLANG_TIDY_EXECUTABLE}"
      "--warnings-as-errors=*"
    )
    set_property(TARGET "${target_name}" PROPERTY CXX_CLANG_TIDY "${clang_tidy_command}")
  endif()

  # Attach cppcheck per-target with settings tuned for this repository.
  if(CPPHL_ENABLE_CPPCHECK)
    find_program(CPPHL_CPPCHECK_EXECUTABLE NAMES cppcheck REQUIRED)

    set(cppcheck_command
      "${CPPHL_CPPCHECK_EXECUTABLE}"
      "--enable=warning,performance,portability,style"
      "--std=c++20"
      "--error-exitcode=2"
      "--inline-suppr"
      "--library=googletest"
      "--suppress=missingIncludeSystem"
    )
    set_property(TARGET "${target_name}" PROPERTY CXX_CPPCHECK "${cppcheck_command}")
  endif()

  # Make static-analysis configuration visible in console output so users can
  # verify that analyzers are attached to each first-party target.
  if(CPPHL_ENABLE_CLANG_TIDY OR CPPHL_ENABLE_CPPCHECK)
    message(STATUS "[static-analysis] target: ${target_name}")
    if(CPPHL_ENABLE_CLANG_TIDY)
      message(STATUS "[static-analysis]   clang-tidy: ${CPPHL_CLANG_TIDY_EXECUTABLE}")
    endif()
    if(CPPHL_ENABLE_CPPCHECK)
      message(STATUS "[static-analysis]   cppcheck: ${CPPHL_CPPCHECK_EXECUTABLE}")
    endif()
  endif()
endfunction()
