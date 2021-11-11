message(STATUS "Projects: M113a benchmark...")

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(sequential) 
add_subdirectory(multicore)
