# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
)

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    ENDIF()
ENDIF()

#--------------------------------------------------------------

SET(PROGRAMS
    directShearJPL
	directShear
	demo_shear
)

FOREACH(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})
ENDFOREACH(PROGRAM)