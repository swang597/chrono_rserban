add_subdirectory(checks)

if(ENABLE_MODULE_IRRLICHT)
  add_subdirectory(sequential)
endif()

if(ENABLE_MODULE_PARALLEL)
  add_subdirectory(parallel)
endif()
