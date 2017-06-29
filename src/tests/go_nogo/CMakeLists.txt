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

SET(GO_NOGO_FILES
    hmmwv_granular.cpp
    driver_model.h
    event_queue.h
)

SOURCE_GROUP("" FILES ${GO_NOGO_FILES})

# ------------------------------------------------------------------------------

MESSAGE(STATUS "HMMWV GO/NO-GO program...")
MESSAGE(STATUS "...add go_nogo")

ADD_EXECUTABLE(go_nogo  ${GO_NOGO_FILES})

SET_TARGET_PROPERTIES(go_nogo PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(go_nogo ${LIBRARIES})
