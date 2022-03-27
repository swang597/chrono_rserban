# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

if (NOT ENABLE_MODULE_OPENGL)
  return()
endif()

INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

set(PROGRAM test_BCE_gen)
source_group(""  FILES "${PROGRAM}.cpp")

set(TINYOBJ_FILES
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
  )
source_group(tiny_obj FILES ${TINYOBJ_FILES})

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp" ${TINYOBJ_FILES})

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_opengl)

