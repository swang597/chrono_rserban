if(NOT ENABLE_MODULE_IRRLICHT)
  return()
endif()  

# ------------------------------------------------------------------------------

set(PROGRAMS
    test_CTC_hulls
    test_CTC_mesh
    test_CTC_box_mesh
    test_CTC_penalty
    test_CTC_box_collision_seq
    test_CTC_rolling_seq
    test_CTC_sentinel
    test_CTC_ANCFshell
)

if(THRUST_FOUND)
    set(PROGRAMS ${PROGRAMS}
        test_CTC_box_triangle_collision
        test_CTC_box_triangle_collision_2
    )
endif()

set(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
set(LIBRARIES ChronoEngine ChronoEngine_irrlicht)

include_directories(${CH_IRRLICHT_INCLUDES})

if(ENABLE_MODULE_PARDISO_MKL)
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LIBRARIES ${LIBRARIES} ChronoEngine_pardisomkl)
  include_directories(${CH_MKL_INCLUDES})
endif()

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")
    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES})
endforeach(PROGRAM)
