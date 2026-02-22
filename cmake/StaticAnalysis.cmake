function(cpphl_apply_static_analysis target_name)
  if(CPPHL_ENABLE_CLANG_TIDY)
    find_program(CPPHL_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(clang_tidy_command
      "${CPPHL_CLANG_TIDY_EXECUTABLE}"
      "--warnings-as-errors=*"
    )
    set_property(TARGET "${target_name}" PROPERTY CXX_CLANG_TIDY "${clang_tidy_command}")
  endif()

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
endfunction()
