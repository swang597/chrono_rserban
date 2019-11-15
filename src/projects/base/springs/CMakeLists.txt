if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_MKL OR NOT ENABLE_MODULE_POSTPROCESS)
  return()
endif()
 

# ------------------------------------------------------------------------------

set(DEMOS
    test_BASE_active_spring
    test_BASE_active_shock
)

include_directories(${CH_IRRLICHTINC})
include_directories(${CH_MKL_INCLUDES})

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_irrlicht ChronoEngine_mkl ChronoEngine_postprocess)
endforeach(PROGRAM)
