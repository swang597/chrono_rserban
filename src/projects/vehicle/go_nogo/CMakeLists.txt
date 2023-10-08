if(NOT ENABLE_MODULE_MULTICORE)
  return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    ENDIF()
ENDIF()

# ------------------------------------------------------------------------------

SET(PROGRAMS
    test_VEH_gonogo_hmmwv
    test_VEH_gonogo_hmmwv_mp
    #test_VEH_gonogo_wvp_mp
)

SET(COMMON_FILES
    driver_model.h
    event_queue.h
    prog_arguments.cpp
    prog_display.cpp
)

# ------------------------------------------------------------------------------

FOREACH(PROG ${PROGRAMS})

    MESSAGE(STATUS "...add ${PROG}")

    ADD_EXECUTABLE(${PROG}  ${PROG}.cpp ${COMMON_FILES})
    SOURCE_GROUP("" FILES ${PROG}.cpp ${COMMON_FILES})
    SET_TARGET_PROPERTIES(${PROG} PROPERTIES
                          FOLDER tests
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${PROG} ${LIBRARIES})

ENDFOREACH()
