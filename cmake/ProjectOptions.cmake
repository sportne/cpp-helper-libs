function(cpphl_configure_project_options)
  add_library(cpphl_project_options INTERFACE)
  target_compile_features(cpphl_project_options INTERFACE cxx_std_20)
endfunction()
