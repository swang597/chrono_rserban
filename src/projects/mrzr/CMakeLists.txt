message(STATUS "Projects: Polaris MRZR...")

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()  

add_subdirectory(basic)
#add_subdirectory(cosim)
