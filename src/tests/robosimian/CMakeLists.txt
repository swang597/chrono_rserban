# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${CH_IRRLICHTINC})
include_directories(${CH_OPENGL_INCLUDES})

set(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
set(LIBRARIES ChronoEngine ChronoEngine_parallel)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS
    test_robosimian
    test_robosimian_par
)

set(MODEL_FILES
    robosimian.h
    robosimian.cpp
)


# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Test programs for RoboSimian...")

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp" ${MODEL_FILES})
    source_group(""  FILES "${PROGRAM}.cpp" ${MODEL_FILES})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_parallel
                          ChronoEngine_irrlicht
                          ChronoEngine_opengl)

endforeach(PROGRAM)

