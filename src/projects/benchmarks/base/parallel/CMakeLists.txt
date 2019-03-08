# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_OPENGL)
    return()
endif()

if(NOT ENABLE_MODULE_PARALLEL)
    return()
endif()

# ------------------------------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

include_directories(${CH_OPENGL_INCLUDES})
list(APPEND LIBS "ChronoEngine_opengl")

include_directories(${CH_PARALLEL_INCLUDES})
set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_PARALLEL_CXX_FLAGS}")
list(APPEND LIBS "ChronoEngine_parallel")

#--------------------------------------------------------------

include_directories(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include)
list(APPEND LIBS gtest_main)

include_directories(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googlebenchmark/include)
list(APPEND LIBS benchmark_main)

# ------------------------------------------------------------------------------

set(TESTS
	test_BM_mixerNSCp
	test_BM_mixerSMCp
)

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )
    target_link_libraries(${PROGRAM} ${LIBS})
endforeach(PROGRAM)
