function(cpphl_configure_coverage)
  # Interface target for optional coverage instrumentation flags.
  add_library(cpphl_coverage INTERFACE)

  if(NOT CPPHL_ENABLE_COVERAGE)
    return()
  endif()

  # This project's coverage workflow is currently implemented with gcov/gcovr,
  # so we require GNU toolchains when coverage is enabled.
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR
      "CPPHL_ENABLE_COVERAGE requires GNU compilers in this project. "
      "Use the gcc-coverage preset or disable coverage.")
  endif()

  # Add compile/link flags that emit coverage data files (.gcno/.gcda).
  target_compile_options(cpphl_coverage INTERFACE --coverage -O0 -g)
  target_link_options(cpphl_coverage INTERFACE --coverage)
endfunction()
