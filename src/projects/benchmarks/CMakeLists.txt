if (NOT BUILD_BENCHMARKING)
  return()
endif()

message(STATUS "Projects: benchmark tests...")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(base/sequential)
add_subdirectory(base/multicore)
add_subdirectory(fea)
#add_subdirectory(test)
