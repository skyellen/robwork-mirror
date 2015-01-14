SET(SUBSYS_NAME robolabFT )
SET(SUBSYS_DESC "Driver for robolab force torque device" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   # INCLUDE_DIRECTORIES( ${robolabFT_INCLUDE_DIR} )
    
    SET(SRC_FILES RobolabFTDriver.cpp)
    SET(SRC_FILES_HPP RobolabFTDriver.hpp)
    
    RW_ADD_LIBRARY(rwhw_robolabFT robolabFT ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(robolabFT "rwhw/robolabFT" ${SRC_FILES_HPP})
    
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_robolabFT;")
    MESSAGE(STATUS "robolabFT include dir:" ${robolabFT_INCLUDE_DIR})
ENDIF()
