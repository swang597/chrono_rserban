if(NOT ENABLE_MODULE_IRRLICHT)
  return()
endif()  

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES 
    ChronoEngine    
    ChronoEngine_vehicle
    ChronoModels_vehicle
    )

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    set(LIBRARIES ${LIBRARIES} ChronoEngine_irrlicht)
endif()

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    set(LIBRARIES ${LIBRARIES} ChronoEngine_mkl)
endif()

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

SET(DEMOS
    test_VEH_quarter_vehicle
    test_VEH_tire_rig
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(DEMO ${DEMOS})

    message(STATUS "...add ${DEMO}")

    add_executable(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
    source_group("" FILES ${DEMO}.cpp)
    set_target_properties(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${DEMO} ${LIBRARIES})
    
endforeach()

