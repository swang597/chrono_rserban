MESSAGE(STATUS "Test programs for HDF5 support...")

if(NOT HDF5_FOUND)
  return()
endif()

add_subdirectory(sequential)
add_subdirectory(parallel)
