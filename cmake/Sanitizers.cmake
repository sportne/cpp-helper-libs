function(cpphl_configure_sanitizers)
  add_library(cpphl_sanitizers INTERFACE)

  if(CPPHL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(cpphl_sanitizers INTERFACE
      $<$<CONFIG:Debug,RelWithDebInfo>:-fsanitize=address,undefined>
      $<$<CONFIG:Debug,RelWithDebInfo>:-fno-omit-frame-pointer>
    )
    target_link_options(cpphl_sanitizers INTERFACE
      $<$<CONFIG:Debug,RelWithDebInfo>:-fsanitize=address,undefined>
    )
  endif()
endfunction()
