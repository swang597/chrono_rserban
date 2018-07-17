if(NOT ENABLE_MODULE_PARALLEL)
  return()
endif()  

message(STATUS "Test programs for PARALLEL module...")

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_PARALLEL_INCLUDES})

set(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
)

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       set (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    endif()
endif()

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
set(DEMOS
    test_PAR_rotate_gravity
    test_PAR_incline_particles
    test_PAR_global_aabb
)

# Programs that require OpenGL
set(DEMOS_GL
    test_PAR_box_granular
    test_PAR_collide_flag
    test_PAR_rot_motor
    test_PAR_callbackNSC
    test_PAR_triangle
)

if(ENABLE_MODULE_OPENGL)
    set(DEMOS ${DEMOS} ${DEMOS_GL})
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)

