# Co-simulation tire test rig
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_MULTICORE)
  return()
endif()  

# ------------------------------------------------------------------------------

include_directories(${CH_INCLUDES})
include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle
    ChronoEngine_multicore
)

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  set(LIBRARIES ${LIBRARIES} ChronoEngine_pardisomkl)
endif()

if(ENABLE_MODULE_MUMPS)
  include_directories(${CH_MUMPS_INCLUDES})
  set(LIBRARIES ${LIBRARIES} ChronoEngine_mumps)
endif()

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

set(PROGRAM test_COSIM_rig)

set(TEST_FILES 
    BaseNode.h
    BaseNode.cpp
    RigNode.h
    RigNode.cpp
    TerrainNode.h
    TerrainNode.cpp)

source_group("" FILES ${TEST_FILES})

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp ${TEST_FILES})

set_target_properties(${PROGRAM} PROPERTIES 
    COMPILE_FLAGS "${COMPILER_FLAGS}"
    LINK_FLAGS "${LINKER_FLAGS}")

target_link_libraries(${PROGRAM} ${LIBRARIES})
