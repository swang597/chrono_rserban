# Test programs for RoboSimian simulation
# ------------------------------------------------------------------------------

message(STATUS "Projects: Robosimian...")

if(NOT ENABLE_MODULE_ROBOSIMIAN OR NOT ENABLE_MODULE_VEHICLE)
  message("  Warning: the Robosimian projects require Chrono::Vehicle!")
  return()
endif()

add_subdirectory(sequential)
add_subdirectory(parallel)
