SET(SUBSYS_NAME tactile )
SET(SUBSYS_DESC "Tactile array sensor library" )
SET(SUBSYS_DEPS rw serialport)

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    SET(SRC_CPP )
    SET(SRC_HPP )
    
    LIST(APPEND SRC_CPP DSACON32.cpp TactileMatrix.cpp TactileMaskMatrix.cpp ConvertUtil.cpp)
    LIST(APPEND SRC_HPP DSACON32.hpp TactileMatrix.hpp TactileMaskMatrix.hpp ConvertUtil.hpp)
    
    RW_ADD_LIBRARY(rwhw_tactile tactile ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_tactile PUBLIC ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(tactile "rwhw/tactile" ${SRC_HPP})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_tactile;")
ENDIF()
