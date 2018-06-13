#=============================================================================
# CMake configuration file for the RigidTerrain demos.
# These example programs requires Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all HMMWV demos

SET(DEMOS
    demo_VEH_RigidTerrain)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executables

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    IF(APPLE)
        ADD_EXECUTABLE(${DEMO} MACOSX_BUNDLE ${DEMO}.cpp)
    ELSE()
        ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
    ENDIF()
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${DEMO}
                          ChronoEngine
                          ChronoEngine_irrlicht
                          ChronoEngine_vehicle
                          ChronoModels_vehicle)
    INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()