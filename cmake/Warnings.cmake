function(cpphl_configure_warnings)
  add_library(cpphl_warnings INTERFACE)

  if(CPPHL_ENABLE_WARNINGS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(cpphl_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
      )
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(cpphl_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
      )
    elseif(MSVC)
      target_compile_options(cpphl_warnings INTERFACE /W4 /permissive-)
    endif()
  endif()
endfunction()
