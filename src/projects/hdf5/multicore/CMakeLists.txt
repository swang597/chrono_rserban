if(NOT ENABLE_MODULE_MULTICORE OR NOT ENABLE_MODULE_OPENGL)
  return()
endif()  

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_HDF5_mcore
)

# ------------------------------------------------------------------------------

include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${CH_OPENGL_INCLUDES})
include_directories(${HDF5_INCLUDE_DIRS})

set(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
set(LIBRARIES ChronoEngine ChronoEngine_multicore ChronoEngine_opengl ${HDF5_CXX_LIBRARIES})


# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CXX_FLAGS}"
        COMPILE_DEFINITIONS "${HDF5_COMPILE_DEFS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)

