SET(SUBSYS_NAME schunkpg70 )
SET(SUBSYS_DESC "Driver for schunk gripper pg70" )
SET(SUBSYS_DEPS rwhw_pcube rwhw_serialport rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    SET(SRC_CPP SchunkPG70.cpp )
    SET(SRC_HPP SchunkPG70.hpp )
    
    RW_ADD_LIBRARY(rwhw_schunkpg70 schunkpg70 ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_schunkpg70 rwhw_pcube rwhw_serialport ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(schunkpg70 "rwhw/schunkpg70" ${SRC_HPP})
    
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_schunkpg70;")
ENDIF()
