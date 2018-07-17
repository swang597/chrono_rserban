message(STATUS "Test programs for WVP...")

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()  

add_subdirectory(checks)
add_subdirectory(sequential)
add_subdirectory(parallel)
