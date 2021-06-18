#--------------------------------------------------------------
# Files for comparing SynChrono scaling performance to baseline Chrono::Vehicle performance
#
# Requires the SynChrono and Chrono::Vehicle modules 
#--------------------------------------------------------------

message(STATUS "Projects: SCM scaling...")

if(NOT ENABLE_MODULE_VEHICLE OR NOT ENABLE_MODULE_SYNCHRONO OR NOT ENABLE_MODULE_IRRLICHT)
  return()
endif()  

include_directories(${CH_IRRLICHTINC})

set(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoEngine_vehicle_irrlicht
    ChronoEngine_synchrono
    )

set(DEMOS
  test_SYN_scaling
  test_VEH_scaling
  )


include_directories(${SYN_INCLUDES})

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${SYN_CXX_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)

