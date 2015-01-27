SET(SUBSYS_NAME netft )
SET(SUBSYS_DESC "driver for netft" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
SET(default TRUE)


FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT BOOST_FOUND)
    set(DEFAULT false)
    set(REASON "Boost system and/or regex not found!")
endif()
 
SET(build TRUE)
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )


IF( build )
    #MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex)
    
    SET(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    SET(SRC_HPP NetFTCommon.hpp NetFTLogging.hpp FTCompensation.hpp)
    
    RW_ADD_LIBRARY(rwhw_netft netft ${SRC_CPP} ${SRC_HPP})
    RW_ADD_INCLUDES(netft "rwhw/netft" ${SRC_HPP})
    ADD_DEPENDENCIES(rwhw_netft ${SUBSYS_DEPS} )    
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_netft;${Boost_LIBRARIES};")
ENDIF()


