if(NOT ENABLE_MODULE_OPENGL)
  return()
endif()

SET(TESTS
  test_GL_meshes
)

INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

message(STATUS "Test programs for OpenGL module...")

foreach(PROGRAM ${TESTS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  target_link_libraries(${PROGRAM}
    ChronoEngine
    ChronoEngine_opengl
    )

endforeach()

