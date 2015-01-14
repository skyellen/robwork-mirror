SET(SUBSYS_NAME trakstar )
SET(SUBSYS_DESC "Calibration of devices" )
SET(SUBSYS_DEPS rw )

FIND_PACKAGE(TrakStar QUIET)

SET(build TRUE)
set(DEFAULT TRAKSTAR_FOUND)
set(REASON "")

IF( ${TRAKSTAR_FOUND} )
    set(REASON "Trakstar library found!")
ELSE()
    set(REASON "Trakstar library not found!")
ENDIF()
IF( ${BUILD_trakstar} )
    FIND_PACKAGE(TrakStar REQUIRED)
    IF( NOT ${TRAKSTAR_FOUND} )
        SET(BUILD_trakstar FALSE)
        set(REASON "Could not find Trakstar libraries!")
    ENDIF()
ENDIF()

RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   
    SET(SRC_CPP )
    SET(SRC_HPP )
    
    LIST(APPEND SRC_CPP Trakstar.cpp)
    LIST(APPEND SRC_HPP Trakstar.hpp)
    
    INCLUDE_DIRECTORIES(${TRAKSTAR_INCLUDE_DIR})
    RW_ADD_LIBRARY(rwhw_trakstar trakstar ${SRC_CPP} ${SRC_HPP})
    RW_ADD_INCLUDES(trakstar "rwhw/trakstar" ${SRC_HPP})
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_trakstar;")
ENDIF()