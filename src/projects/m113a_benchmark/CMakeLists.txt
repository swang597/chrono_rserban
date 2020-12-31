message(STATUS "Projects: M113a benchmark...")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(sequential) 
add_subdirectory(multicore)
