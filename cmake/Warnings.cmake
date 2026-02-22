function(cpphl_configure_warnings)
  # Interface target to centralize warning configuration.
  add_library(cpphl_warnings INTERFACE)

  if(CPPHL_ENABLE_WARNINGS)
    # Choose compiler-appropriate warning sets.
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      target_compile_options(cpphl_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
      )
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(cpphl_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
      )
    elseif(MSVC)
      # /permissive- enforces closer standard conformance on MSVC.
      target_compile_options(cpphl_warnings INTERFACE
        /W4
        /permissive-
        /w14062
        $<$<CONFIG:Debug>:/RTCcsu>
      )
    endif()
  endif()
endfunction()

function(cpphl_apply_warning_policy target_name)
  if(CPPHL_WARNINGS_AS_ERRORS)
    set_property(TARGET "${target_name}" PROPERTY COMPILE_WARNING_AS_ERROR ON)
  endif()
endfunction()
