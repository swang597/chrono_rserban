# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

SET(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
SET(LIBRARIES ChronoEngine ChronoEngine_parallel)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
	test_body_force_SEQ
    test_body_force_PAR
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for body force...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_parallel
                          ChronoEngine_irrlicht
                          ChronoEngine_opengl)

ENDFOREACH(PROGRAM)

