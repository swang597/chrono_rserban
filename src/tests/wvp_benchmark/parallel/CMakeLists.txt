# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
    ChronoEngine_opengl
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()


# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
    test_WVP_parallel
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for WVP (parallel)...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH(PROGRAM)

