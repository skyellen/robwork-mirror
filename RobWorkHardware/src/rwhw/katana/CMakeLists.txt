

FIND_FILE(katana_kmlBase_h "KNI/kmlBase.h" ${KATANA_INCLUDE_DIR})

IF(NOT katana_kmlBase_h)
    MESSAGE(STATUS "Katana disabled! include dir not found!")
ELSE()
    include_directories(${KATANA_INCLUDE_DIR})
    
    include(CheckIncludeFiles)
    
    set(CMAKE_REQUIRED_INCLUDES ${KATANA_INCLUDE_DIR})
    CHECK_INCLUDE_FILES(KNI/cdlCOM.h KATANA_CDLCOM_H)
        
    find_file(katana_cdlCOM_h "KNI/cdlCOM.h" ${KATANA_INCLUDE_DIR} )
    find_file(katana_cplSerial_h "KNI/cplSerial.h" ${KATANA_INCLUDE_DIR})
    
    
    if (NOT katana_cdlCOM_h)
    	message(STATUS "Could not find KNI/cdlCOM.h required to compile Kanata")
    endif()
    
    if (NOT katana_cplSerial_h)
    	message(STATUS "Could not find KNI/cplSerial.h required to compile Katana")
    endif()
    
    if (NOT katana_kmlBase_h)
    	message(STATUS "Could not find KNI/kmlBase.h required to compile Katana")
    endif()
    
    
    if( katana_cdlCOM_h AND katana_cplSerial_h AND katana_kmlBase_h )
        MESSAGE(STATUS "Katana enabled! include dir found!")
        add_library(
          rwhw_katana
          Katana.cpp
        )
    ELSE ()
        MESSAGE(STATUS "Katana disabled! include dir not found!") 
    endif()
ENDIF()