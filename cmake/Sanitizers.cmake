function(cpphl_configure_sanitizers)
  # Interface target for optional runtime memory/UB diagnostics.
  add_library(cpphl_sanitizers INTERFACE)

  if(CPPHL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Enable ASan+UBSan only for debug-like builds to avoid release overhead.
    target_compile_options(cpphl_sanitizers INTERFACE
      $<$<CONFIG:Debug,RelWithDebInfo>:-fsanitize=address,undefined>
      $<$<CONFIG:Debug,RelWithDebInfo>:-fno-omit-frame-pointer>
    )
    target_link_options(cpphl_sanitizers INTERFACE
      $<$<CONFIG:Debug,RelWithDebInfo>:-fsanitize=address,undefined>
    )
  endif()
endfunction()
