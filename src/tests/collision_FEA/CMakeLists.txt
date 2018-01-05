# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_collision_ANCFshell
)

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

MESSAGE(STATUS "Test programs for FEA collision...")

#--------------------------------------------------------------
# For demo executables that use Irrlicht
# (and possibly Matlab and/or MKL)

    
FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")

    TARGET_LINK_LIBRARIES(${PROGRAM}
        ChronoEngine
        ChronoEngine_fea
        ChronoEngine_irrlicht)

ENDFOREACH()


