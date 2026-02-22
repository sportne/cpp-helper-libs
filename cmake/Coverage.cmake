function(cpphl_configure_coverage)
  add_library(cpphl_coverage INTERFACE)

  if(NOT CPPHL_ENABLE_COVERAGE)
    return()
  endif()

  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR
      "CPPHL_ENABLE_COVERAGE requires GNU compilers in this project. "
      "Use the gcc-coverage preset or disable coverage.")
  endif()

  target_compile_options(cpphl_coverage INTERFACE --coverage -O0 -g)
  target_link_options(cpphl_coverage INTERFACE --coverage)
endfunction()
