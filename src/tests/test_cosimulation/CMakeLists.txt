# REQUIRES: FEA, Vehicle, Irrlicht, OpenGL

MESSAGE(STATUS "Test programs for COSIMULATION...")

if(NOT ENABLE_MODULE_IRRLICHT)
   message(STATUS "...requires Chrono::Irrlicht")
   return()
endif()

if(NOT ENABLE_MODULE_OPENGL)
   message(STATUS "...requires Chrono::OpenGL")
   return()
endif()

if(NOT ENABLE_MODULE_VEHICLE)
   message(STATUS "...requires Chrono::Vehicle")
   return()
endif()

if(NOT ENABLE_MODULE_FEA)
   message(STATUS "...requires Chrono::FEA")
   return()
endif()

if(NOT ENABLE_MODULE_MKL)
   message(STATUS "...requires Chrono::MKL")
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
    ChronoEngine_fea
    ChronoEngine_mkl
    ChronoEngine_vehicle
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
    test_cosimulation
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

