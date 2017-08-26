# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
)

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    ENDIF()
ENDIF()

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
    rotate_gravity
    incline_particles
    global_aabb
)

# Programs that require OpenGL
SET(DEMOS_GL
    box_granular
    collide_flag
    jcol_example
)

IF(ENABLE_MODULE_OPENGL)
    SET(DEMOS ${DEMOS} ${DEMOS_GL})
ENDIF()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for PARALLEL module...")

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

