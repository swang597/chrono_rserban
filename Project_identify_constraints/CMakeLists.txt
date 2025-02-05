#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
set(CMAKE_BUILD_TYPE Debug)
 

cmake_minimum_required(VERSION 3.10)

#--------------------------------------------------------------
# === 1 === 
# Set the project name
#--------------------------------------------------------------

project(IdentifyConstraints)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (case insensitive):
#   Cascade, Cosimulation, Irrlicht, OpenGL, Matlab, Multicore, Gpu,
#   PardisoMKL, PardisoProject, Postprocess, Python, Vehicle,
#   VehicleCosimm, VSG.
# A component can be requested either as required or optional
# (see the CMake documentation for find_package).
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the chrono-config.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# Chrono_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------
# set(Chrono_DIR "/home/swang597/Documents/Research/chrono_fork_rserban/build/cmake")#Shu


# LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
# find_package(Chrono
#              COMPONENTS Irrlicht Vehicle
#              OPTIONAL_COMPONENTS
#              CONFIG)

set(Chrono_DIR "/home/swang597/Documents/Research/chrono_fork_rserban/build_sensor/cmake")#Shu             
LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Important! To ensure ABI compatibility, use the same C++ standard
# as the one used to build the Chrono libraries.
#--------------------------------------------------------------

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

set(ML_UTILITY_PATH /home/swang597/Documents/Research/Project_heightmap/Utility_projHM/) #Shu
include_directories(${ML_UTILITY_PATH}) #Shu

# file(GLOB ML_SOURCES "${ML_UTILITY_PATH}/*.cpp")

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()


#Shu:231025

# Add the include directories for LibTorch
include_directories(
    /opt/libtorch/include
    /opt/libtorch/include/torch/csrc/api/include
)

# Specify the location of the LibTorch libraries
link_directories(/opt/libtorch/lib)

# Add the include directories and link directories for OpenCV from Conda
set(OpenCV_INCLUDE_DIRS /home/swang597/Documents/Package/anaconda3/envs/CNN/include)
set(OpenCV_LIBRARY_DIR /home/swang597/Documents/Package/anaconda3/envs/CNN/lib)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})

# Include and link directories for CNPY
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Find OpenCV libraries
find_package(OpenCV REQUIRED)

#Shu.

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------
# add_executable(cpp_double_pendulum cpp_double_pendulum.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(cpp_double_pendulum PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(cpp_slider_crank cpp_slider_crank.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(cpp_slider_crank PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_slider_crank PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_slider_crank PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_slider_crank ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(cpp_toy_car cpp_toy_car.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(cpp_toy_car PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_toy_car PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_toy_car PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_toy_car ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(demo_VEH_SCMTerrain_RigidTire demo_VEH_SCMTerrain_RigidTire.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(demo_VEH_SCMTerrain_RigidTire PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(demo_VEH_SCMTerrain_RigidTire PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(demo_VEH_SCMTerrain_RigidTire PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(demo_VEH_SCMTerrain_RigidTire ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(demo_ROBOT_Iris_Rigid demo_ROBOT_Iris_Rigid.cpp Iris.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(demo_ROBOT_Iris_Rigid PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(demo_ROBOT_Iris_Rigid PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(demo_ROBOT_Iris_Rigid PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(demo_ROBOT_Iris_Rigid ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# # # dump acceleration data from chrono simulation as IMU data
# add_executable(cpp_slider_crank_dumpIMUdata_240913 cpp_slider_crank_dumpIMUdata_240913.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(cpp_slider_crank_dumpIMUdata_240913 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_slider_crank_dumpIMUdata_240913 PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_slider_crank_dumpIMUdata_240913 PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_slider_crank_dumpIMUdata_240913 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# # # dump acceleration data from chrono simulation as IMU data
# add_executable(cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919 cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919.cpp)
# target_compile_definitions(cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919 PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919 PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum_RJ-SJ_dumpIMUdata_240919 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# # dump acceleration data from chrono simulation as IMU data
# add_executable(cpp_double_pendulum_2SJ_dumpIMUdata_240927 cpp_double_pendulum_2SJ_dumpIMUdata_240927.cpp)
# target_compile_definitions(cpp_double_pendulum_2SJ_dumpIMUdata_240927 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum_2SJ_dumpIMUdata_240927 PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum_2SJ_dumpIMUdata_240927 PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum_2SJ_dumpIMUdata_240927 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001 cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001.cpp)
# target_compile_definitions(cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001 PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001 PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum_RJ-SJ_dumpIMUdata_LshapeBody1_241001 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu


# add_executable(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007 cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007.cpp)
# target_compile_definitions(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007 PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007 PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241007 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu


add_executable(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226 cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226.cpp)
target_compile_definitions(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226 PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226 PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226 PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(cpp_double_pendulum_SJ_RJ_dumpIMUdata_241226 ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

# add_executable(demo_SEN_GPSIMU demo_SEN_GPSIMU.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(demo_SEN_GPSIMU PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(demo_SEN_GPSIMU PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(demo_SEN_GPSIMU PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(demo_SEN_GPSIMU ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu


# add_executable(cpp_double_pendulum_IMU cpp_double_pendulum_IMU.cpp ${ML_UTILITY_PATH}TorchModelRunner.cpp ${ML_UTILITY_PATH}Heightmap.cpp)
# target_compile_definitions(cpp_double_pendulum_IMU PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(cpp_double_pendulum_IMU PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(cpp_double_pendulum_IMU PUBLIC ${CHRONO_LINKER_FLAGS})
# target_link_libraries(cpp_double_pendulum_IMU ${CHRONO_LIBRARIES} cnpy torch c10 torch_cpu ${OpenCV_LIBS}) # Shu

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
add_DLL_copy_command()
