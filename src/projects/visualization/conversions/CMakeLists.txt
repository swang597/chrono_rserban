set(PROGRAMS
    test_VIS_XMLtoOBJ
)

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ChronoEngine)

endforeach(PROGRAM)
