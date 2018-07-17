#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib64" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib64")
endif("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

# Enable creation of "application bundles" on MacOSX.
# This is necessary for Irrlicht-based tests and optional for OpenGL or non-graphics tests.
# For simplicity and consistency, we enable here MACOSX_BUNDLE for all subsequent programs.

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------

message(STATUS "==== Test programs ====")





add_subdirectory(core)
add_subdirectory(filesystem)
add_subdirectory(hdf5)

if(ENABLE_MODULE_IRRLICHT)
    add_subdirectory(collision)
    add_subdirectory(render)
    add_subdirectory(steering_mechanism)
endif()

if(ENABLE_MODULE_FEA AND ENABLE_MODULE_IRRLICHT)
    add_subdirectory(collision_FEA)
endif()

if(ENABLE_MODULE_POSTPROCESS)
    add_subdirectory(postprocess)
endif()

if(ENABLE_MODULE_PARALLEL)
    add_subdirectory(parallel)
    add_subdirectory(box_collision)
    add_subdirectory(rolling)
endif()

if(ENABLE_MODULE_PARALLEL AND ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_OPENGL)
    add_subdirectory(body_force)
endif()

if(ENABLE_MODULE_PARALLEL AND ENABLE_MODULE_IRRLICHT)
   add_subdirectory(parallel_irrlicht)
endif()

option(BUILD_TESTS_DISTRIBUTED "Build Chrono::Distributed test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_DISTRIBUTED)
if(BUILD_TESTS_DISTRIBUTED)
  add_subdirectory(distributed)
endif()

option(BUILD_TESTS_COSIMULATION "Build co-simulation test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_COSIMULATION)
if(BUILD_TESTS_COSIMULATION)
  add_subdirectory(cosimulation)
endif()

option(BUILD_TESTS_ROBOSIMIAN "Build RoboSimian test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_ROBOSIMIAN)
if(BUILD_TESTS_ROBOSIMIAN)
  add_subdirectory(robosimian)
endif()

option(BUILD_TESTS_GEOMECHANICS "Build geomechanics test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_GEOMECHANICS)
if(BUILD_TESTS_GEOMECHANICS)
  add_subdirectory(geomechanics)
endif()

option(BUILD_TESTS_WVP "Build WVP (wheeled vehicle platform) test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_WVP)
if(BUILD_TESTS_WVP)
  add_subdirectory(wvp_benchmark)
endif()

option(BUILD_TESTS_VEHICLE "Build Chrono::Vehicle test programs" ON)
mark_as_advanced(FORCE BUILD_TESTS_VEHICLE)
if(BUILD_TESTS_VEHICLE)
  add_subdirectory(vehicle)
endif()
