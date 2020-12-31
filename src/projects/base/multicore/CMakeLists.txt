if(NOT ENABLE_MODULE_MULTICORE)
  return()
endif()

if(NOT ENABLE_MODULE_OPENGL)
  return()
endif()    

# ------------------------------------------------------------------------------

set(DEMOS
    test_BASE_body_force_mcore
    test_BASE_collide_flag_mcore
    test_BASE_rot_motor_mcore
    test_BASE_callbackNSC_mcore
    test_BASE_global_aabb_mcore
    test_BASE_rotate_gravity_mcore
    test_BASE_active_spring_mcore
)

include_directories(${CH_MULTICORE_INCLUDES})
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
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_multicore
                          ChronoEngine_opengl)
endforeach(PROGRAM)
