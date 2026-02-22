function(cpphl_configure_project_options)
  # Interface target lets us attach project-wide compile requirements once
  # and link them to any library/executable that should inherit them.
  add_library(cpphl_project_options INTERFACE)

  # Require C++20 on every target that links this interface library.
  target_compile_features(cpphl_project_options INTERFACE cxx_std_20)

  # Enable glibc fortify checks for optimized release-like configurations on
  # GNU/Clang toolchains. Debug builds intentionally remain unaffected.
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_definitions(cpphl_project_options INTERFACE
      $<$<CONFIG:Release,RelWithDebInfo,MinSizeRel>:_FORTIFY_SOURCE=2>
    )
  endif()
endfunction()
