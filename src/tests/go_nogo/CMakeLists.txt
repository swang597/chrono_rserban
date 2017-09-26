# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
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
    gonogo_hmmwv
    gonogo_wvp_mp
)

SET(COMMON_FILES
    driver_model.h
    event_queue.h
    prog_arguments.cpp
    prog_display.cpp
)

# ------------------------------------------------------------------------------

MESSAGE(STATUS "GO/NO-GO programs...")

FOREACH(PROG ${PROGRAMS})

    MESSAGE(STATUS "...add ${PROG}")

    ADD_EXECUTABLE(${PROG}  ${PROG}.cpp ${COMMON_FILES})
    SOURCE_GROUP("" FILES ${PROG}.cpp ${COMMON_FILES})
    SET_TARGET_PROPERTIES(${PROG} PROPERTIES
                          FOLDER tests
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${PROG} ${LIBRARIES})

ENDFOREACH()
