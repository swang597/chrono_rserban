#--------------------------------------------------------------

set(POST_DEMOS
	cubic_spline
	)

#--------------------------------------------------------------

message(STATUS "Test programs for POSTPROCESS module...")

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
