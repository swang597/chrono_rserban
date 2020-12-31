# Test programs for multicore RoboSimian simulations
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_MULTICORE OR NOT ENABLE_MODULE_OPENGL)
  message("  Warning: the multicore Robosimian projects require Chrono::Multicoreand Chrono::OpenGL!")
  return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS
    test_RS_robosimian_Granular
    test_RS_terrain_Granular
)

set(UTILITIES
    granular.h
    granular.cpp
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp" ${UTILITIES})
    source_group(""  FILES "${PROGRAM}.cpp" ${UTILITIES})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_multicore
                          ChronoEngine_vehicle
                          ChronoEngine_opengl
                          ChronoModels_robosimian)
endforeach()
