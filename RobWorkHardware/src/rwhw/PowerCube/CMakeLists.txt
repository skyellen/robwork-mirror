SET(SUBSYS_NAME pcube )
SET(SUBSYS_DESC "Driver for powercube modules" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(COMPONENT_${SUBSYS_NAME}_ENABLE "Enable ${SUBSYS_NAME} component." 
                       ${COMPONENT_${SUBSYS_NAME}_ENABLE} "COMPONENT_${SUBSYS_NAME}_ENABLE" OFF)

set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} COMPONENT_${SUBSYS_NAME}_ENABLE ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")    

    SET(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
    SET(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)

    RW_ADD_LIBRARY(rwhw_pcube pcube ${SRC_CPP} ${SRC_HPP})
    RW_ADD_INCLUDES(pcube "rwhw/PowerCube" ${SRC_HPP})
    set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_pcube;")
ENDIF()