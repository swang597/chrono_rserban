# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

if (NOT ENABLE_MODULE_OPENGL)
  return()
endif()

include_directories(${CH_OPENGL_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   set (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

set(PROGRAMS 
   test_UTILS_BCE_gen
   test_UTILS_Particle_gen
   )

set(TINYOBJ_FILES
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
  ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
  )
source_group(tiny_obj FILES ${TINYOBJ_FILES})

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")
    
    add_executable(${PROGRAM}  "${PROGRAM}.cpp" ${TINYOBJ_FILES})
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES
      FOLDER demos
      COMPILE_FLAGS "${CH_CXX_FLAGS}"
      LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_opengl)
endforeach()
