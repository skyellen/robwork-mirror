SET(SUBSYS_NAME sdh )
SET(SUBSYS_DESC "Driver for robolab sdh " )
SET(SUBSYS_DEPS rw )

# make sure that SDH can be found
SET(build TRUE)

FIND_PACKAGE(SDH QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SDH_FOUND)
    set(DEFAULT false)
    set(REASON "SDH library not found!")
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    FIND_PACKAGE(SDH)
    INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )

    FIND_PACKAGE(PEAKCAN)
    
    SET(SRC_FILES )
    SET(SRC_FILES_HPP )
    IF(TOLUA++_FOUND)
    #    ADD_TOLUA_PACKAGE("SDHDriverLua" "SDHDriverLuaStub")
    ENDIF()
        
    RW_ADD_LIBRARY(rwhw_sdh sdh SDHDriver.cpp SDHDriver.hpp ${SRC_FILES} ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(rwhw_sdh ${ROBWORK_LIBRARIES} ${SDH_LIBRARY} ${PEAKCAN_LIBRARIES})
    RW_ADD_INCLUDES(sdh "rwhw/sdh" SDHDriver.hpp SDHDriverLua.hpp )
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_sdh;")
    
ENDIF()
