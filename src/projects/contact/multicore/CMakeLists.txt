if(NOT ENABLE_MODULE_MULTICORE)
  return()
endif()  

if(NOT ENABLE_MODULE_OPENGL)
  return()
endif()  

# ------------------------------------------------------------------------------

set(PROGRAMS
    test_CTC_box_collision_mcore
    test_CTC_rolling_mcore
    test_CTC_mesh_mcore
)

include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

set(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
set(LIBRARIES ChronoEngine ChronoEngine_multicore ChronoEngine_opengl)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)

