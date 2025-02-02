# Co-simulation full vehicle (HMMWV)
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_MKL)
  return()
endif()

if(NOT ENABLE_MODULE_MULTICORE)
  return()
endif()

# ------------------------------------------------------------------------------

include_directories(${CH_MKL_INCLUDES})
include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle
    ChronoEngine_mkl
    ChronoEngine_multicore
)

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       set(LINKER_FLAGS  "${LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
    endif()
endif()

if(ENABLE_MODULE_DISTRIBUTED)
    include_directories(${CH_DISTRIBUTED_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_distributed)
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_DISTRIBUTED_LINK_FLAGS}")
endif()

# ------------------------------------------------------------------------------

set(PROGRAM test_COSIM_check_sampler)
message(STATUS "...add ${PROGRAM}")
add_executable(${PROGRAM} ${PROGRAM}.cpp)    
source_group("" FILES ${PROGRAM}.cpp)
set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(${PROGRAM} ${LIBRARIES})


set(COMMON_FILES 
    BaseNode.h
    BaseNode.cpp
    VehicleNode.h
    VehicleNode.cpp
    TireNode.h
    TireNode.cpp
    )

set(PROGRAM test_COSIM_check_triangles)
set(TEST_FILES "${COMMON_FILES}")
message(STATUS "...add ${PROGRAM}")
add_executable(${PROGRAM} ${PROGRAM}.cpp ${TEST_FILES})    
source_group("" FILES ${PROGRAM}.cpp)
set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(${PROGRAM} ${LIBRARIES})


set(PROGRAM test_COSIM_vehicle)
set(TEST_FILES "${COMMON_FILES}" TerrainNode.h TerrainNode.cpp)
message(STATUS "...add ${PROGRAM}")
add_executable(${PROGRAM} ${PROGRAM}.cpp ${TEST_FILES})    
source_group("" FILES ${PROGRAM}.cpp ${TEST_FILES})
set_target_properties(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(${PROGRAM} ${LIBRARIES})


if(ENABLE_MODULE_DISTRIBUTED)
    set(PROGRAM test_COSIM_vehicle_distr)
    set(TEST_FILES "${COMMON_FILES}" TerrainNodeDistr.h TerrainNodeDistr.cpp)
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} ${PROGRAM}.cpp ${TEST_FILES})    
    source_group("" FILES ${PROGRAM}.cpp ${TEST_FILES})
    set_target_properties(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${PROGRAM} ${LIBRARIES})
endif()
