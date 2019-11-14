#=============================================================================
# CMake configuration file for the Chrono Robosimian module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

MESSAGE(STATUS "Model of the ROBOSIMIAN robot...")

#-----------------------------------------------------------------------------
# List the files in the ChronoModels_robosimian library

SET(ChronoModels_robosimian_SOURCES 
    robosimian.cpp
)

SET(ChronoModels_robosimian_HEADERS
    ../ChApiModels.h
	robosimian.h 
	driver_cb.h
)

SOURCE_GROUP("" FILES 
            ${ChronoModels_robosimian_SOURCES} 
            ${ChronoModels_robosimian_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The ROBOSIMIAN library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoModels_robosimian SHARED 
			${ChronoModels_robosimian_SOURCES}
			${ChronoModels_robosimian_HEADERS})

SET_TARGET_PROPERTIES(ChronoModels_robosimian PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
					  COMPILE_DEFINITIONS "CH_API_COMPILE_MODELS")
                          
TARGET_LINK_LIBRARIES(ChronoModels_robosimian 
	                  ChronoEngine)
	
ADD_DEPENDENCIES(ChronoModels_robosimian ChronoEngine)
	
INSTALL(TARGETS ChronoModels_robosimian
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

INSTALL(FILES ${ChronoModels_robosimian_HEADERS}
        DESTINATION include/chrono_robosimian)


