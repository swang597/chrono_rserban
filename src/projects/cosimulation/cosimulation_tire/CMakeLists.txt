# REQUIRES: MKL, Vehicle, Irrlicht, OpenGL

if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_OPENGL)
   return()
endif()

if(NOT ENABLE_MODULE_VEHICLE)
   return()
endif()

if(NOT ENABLE_MODULE_MKL)
   return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_opengl
    ChronoEngine_mkl
    ChronoEngine_vehicle
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
    test_COSIM_tire
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH(PROGRAM)

