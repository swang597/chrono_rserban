MESSAGE(STATUS "Projects: FEA module...")

if(NOT ENABLE_MODULE_IRRLICHT OR NOT ENABLE_MODULE_MKL)
  return()
endif()  

#--------------------------------------------------------------

SET(FEA_DEMOS_IRR_MKL
    test_FEA_ancf_cable
)

INCLUDE_DIRECTORIES(CMAKE_CURRENT_SOURCE_DIR)
INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})


#--------------------------------------------------------------

FOREACH(PROGRAM ${FEA_DEMOS_IRR_MKL})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")   

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MKL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
        
    TARGET_LINK_LIBRARIES(${PROGRAM}
        ${CH_IRRLICHTLIB}
        ChronoEngine
        ChronoEngine_mkl
        ChronoEngine_irrlicht)

ENDFOREACH()






