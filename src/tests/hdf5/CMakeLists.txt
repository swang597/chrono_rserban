MESSAGE(STATUS "Test programs for HDF5 support...")

# ------------------------------------------------------------------------------
# Find HDF5 library (required)
# ------------------------------------------------------------------------------
# LIB_TYPE can be one of STATIC or SHARED.
# If using shared libraries on Windows, MUST define H5_BUILT_AS_DYNAMIC_LIB to 
# have the symbols imported from the DLL.
# Note also that the path to the HDF5 DLLs must be in the search path (or else
# copied in the working directory).

set (LIB_TYPE SHARED) # STATIC or SHARED
string(TOLOWER ${LIB_TYPE} SEARCH_TYPE)
find_package (HDF5 NAMES hdf5 COMPONENTS CXX ${SEARCH_TYPE})

if(NOT HDF5_FOUND)
    message(STATUS "Cannot find HDF5 library")
    return()
endif()

message(STATUS "HDF5 include dir: ${HDF5_INCLUDE_DIR}")
message(STATUS "HDF5 C library:   ${HDF5_C_${LIB_TYPE}_LIBRARY}")
message(STATUS "HDF5 C++ library: ${HDF5_CXX_${LIB_TYPE}_LIBRARY}")

if (ENABLE_MODULE_IRRLICHT)
  add_subdirectory(sequential)
endif()

if (ENABLE_MODULE_PARALLEL AND ENABLE_MODULE_OPENGL)
  add_subdirectory(parallel)
endif()  
