# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
    test_HDF5_seq
)

# ------------------------------------------------------------------------------

include_directories(${CH_IRRLICHTINC})
include_directories(${HDF5_INCLUDE_DIR})

set(CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
set(LIBRARIES ChronoEngine ChronoEngine_irrlicht ${HDF5_CXX_${LIB_TYPE}_LIBRARY})


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
        COMPILE_DEFINITIONS "H5_BUILT_AS_DYNAMIC_LIB "
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach(PROGRAM)

