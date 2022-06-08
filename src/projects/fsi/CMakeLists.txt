message(STATUS "Projects: FSI...")

if(NOT ENABLE_MODULE_FSI)
  return()
endif()  


# List FSI demos that do not have other dependencies
SET(FSI_DEMOS 
    test_FSI_WheelDrop
)

#--------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_FSI_INCLUDES})

MESSAGE(STATUS "Demo programs for FSI module...")

FOREACH(PROGRAM ${FSI_DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_fsi)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
