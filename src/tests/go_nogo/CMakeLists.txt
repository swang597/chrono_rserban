# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
    ChronoEngine_opengl
    ChronoEngine_vehicle
    ChronoModels_vehicle
)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()


# ------------------------------------------------------------------------------

SET(GO_NOGO_FILES
    hmmwv_granular.cpp
    driver_model.h
    monitoring.h
    event_queue.h
)

SOURCE_GROUP("" FILES "${GO_NOGO_FILES}")
# ------------------------------------------------------------------------------

MESSAGE(STATUS "GO/NO-GO program...")

ADD_EXECUTABLE(go_nogo  "${GO_NOGO_FILES}")

SET_TARGET_PROPERTIES(go_nogo PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(go_nogo ${LIBRARIES})
