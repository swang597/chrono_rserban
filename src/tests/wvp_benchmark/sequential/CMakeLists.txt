# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_WVP_sequential
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Test programs for WVP (sequential)...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH(PROGRAM)

