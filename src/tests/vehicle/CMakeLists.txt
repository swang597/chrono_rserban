message(STATUS "Test programs for VEHICLE module...")

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()  

add_subdirectory(sequential)
add_subdirectory(go_nogo)
add_subdirectory(go_nogo_SCM)
