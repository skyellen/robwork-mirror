SET(SUBSYS_NAME sdh )
SET(SUBSYS_DESC "Driver for robolab sdh " )
SET(SUBSYS_DEPS rw )

# make sure that SDH can be found
SET(build TRUE)
set(DEFAULT FALSE)
set(REASON) 
IF( ${BUILD_sdh} )
    FIND_PACKAGE(SDH)
    IF( NOT SDH_FOUND )
        #MESSAGE(STATUS "NOT FOUND")
        set(DEFAULT FALSE)
        set(BUILD_sdh FALSE)
        set(REASON "SDH library not found!")
    ENDIF()
ENDIF()

RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} COMPONENT_${SUBSYS_NAME}_ENABLE ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   
    INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )
    
    SET(SRC_FILES )
    SET(SRC_FILES_HPP )
    IF(TOLUA++_FOUND)
    #    ADD_TOLUA_PACKAGE("SDHDriverLua" "SDHDriverLuaStub")
    ENDIF()
        
    RW_ADD_LIBRARY(rwhw_sdh sdh SDHDriver.cpp SDHDriver.hpp ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(sdh "rw/hw" SDHDriver.hpp SDHDriverLua.hpp )
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_sdh;")
    
ENDIF()
