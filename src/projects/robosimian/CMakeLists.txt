# Test programs for RoboSimian simulation
# ------------------------------------------------------------------------------

message(STATUS "Projects: RoboSimian...")

if(NOT ENABLE_MODULE_PARALLEL OR NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_OPENGL)
  return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${CH_IRRLICHTINC})
include_directories(${CH_OPENGL_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS_seq
    test_RS_robosimian_Rigid
    test_RS_robosimian_SCM
)

set(PROGRAMS_par
    test_RS_robosimian_Granular
    test_RS_terrain_Granular
)

set(MODEL_FILES
    robosimian.h
    robosimian.cpp
    driver_cb.h
)

set(MODEL_FILES_seq
    ${MODEL_FILES}
    irrlicht_app.h
    irrlicht_app.cpp
)

set(MODEL_FILES_par
    ${MODEL_FILES}
    granular.h
    granular.cpp
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${PROGRAMS_seq})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp" ${MODEL_FILES_seq})
    source_group(""  FILES "${PROGRAM}.cpp" ${MODEL_FILES_seq})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_vehicle
                          ChronoEngine_opengl)
endforeach()

foreach(PROGRAM ${PROGRAMS_par})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp" ${MODEL_FILES_par})
    source_group(""  FILES "${PROGRAM}.cpp" ${MODEL_FILES_par})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_parallel
                          ChronoEngine_vehicle
                          ChronoEngine_opengl)
endforeach()
