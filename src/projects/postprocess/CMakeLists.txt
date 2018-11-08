message(STATUS "Projects: Chrono::Postprocess module...")

if(NOT ENABLE_MODULE_POSTPROCESS)
  return()
endif()  

#--------------------------------------------------------------

set(POST_DEMOS
	test_POST_cubic_spline
	)

foreach(PROGRAM ${POST_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_postprocess)
    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_postprocess)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
