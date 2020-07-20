#=============================================================================
# M113 parallel test and validation programs
#=============================================================================

if(NOT ENABLE_MODULE_PARALLEL)
  return()
endif()

# -------------------------------------------------------------
# Additional include paths and libraries
# -------------------------------------------------------------

include_directories(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
endif()

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

set(TERRAIN_FILES
  ../terrain/RigidTerrainSlope.h
  ../terrain/RigidTerrainSlope.cpp
  ../terrain/RigidTerrainStep.h
  ../terrain/RigidTerrainStep.cpp
  ../terrain/RigidTerrainTrapezoid.h
  ../terrain/RigidTerrainTrapezoid.cpp
)

SET(DEMOS
  test_M113a_granular
  test_M113a_granular_accel
  test_M113a_offRoadTrafficability_granular
)

#--------------------------------------------------------------
# Add all executables
#--------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp ${TERRAIN_FILES})
  source_group("terrain" FILES ${TERRAIN_FILES})
  source_group(""  FILES ${PROGRAM}.cpp ${TERRAIN_FILES})

  set_target_properties(${PROGRAM} PROPERTIES
    FOLDER tests
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
    LINK_FLAGS "${CHRONO_LINKERFLAG_EXE}"
  )

  target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)
