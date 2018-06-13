# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_render_line
)

# ------------------------------------------------------------------------------

SET(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
SET(LIBRARIES ChronoEngine ChronoEngine_irrlicht)

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})


# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for rendering...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    IF(APPLE)
        ADD_EXECUTABLE(${PROGRAM} MACOSX_BUNDLE "${PROGRAM}.cpp")
    ELSE()
        ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    ENDIF()
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH(PROGRAM)

