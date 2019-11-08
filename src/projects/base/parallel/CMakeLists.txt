if(NOT ENABLE_MODULE_PARALLEL)
  return()
endif()

if(NOT ENABLE_MODULE_OPENGL)
  return()
endif()    

# ------------------------------------------------------------------------------

set(DEMOS
    test_BASE_body_force_par
    test_BASE_collide_flag_par
    test_BASE_rot_motor_par
    test_BASE_callbackNSC_par
    test_BASE_global_aabb_par
    test_BASE_rotate_gravity_par
    test_BASE_active_spring_par
)

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_parallel
                          ChronoEngine_opengl)
endforeach(PROGRAM)
