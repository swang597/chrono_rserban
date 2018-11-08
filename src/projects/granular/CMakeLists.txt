message(STATUS "Projects: granular dynamics...")

if(NOT ENABLE_MODULE_PARALLEL)
  return()
endif()  

if(NOT ENABLE_MODULE_OPENGL)
  return()
endif()    

# ------------------------------------------------------------------------------

set(DEMOS
    test_GRAN_incline_particles
    test_GRAN_box_granular
    test_GRAN_triangle
)

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_parallel ChronoEngine_opengl)

endforeach(PROGRAM)
