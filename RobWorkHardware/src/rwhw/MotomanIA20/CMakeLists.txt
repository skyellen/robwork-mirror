SET(SUBSYS_NAME motomanIA20 )
SET(SUBSYS_DESC "Library for controlling the motomanIA20 robot" )
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
    IF (NOT DEFINED WIN32)
        ADD_LIBRARY(
          rwhw_motomanIA20
          MotomanIA20.cpp
        )
        SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_motomanIA20;")
        MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component DISABLED - linux only")
    ENDIF()
ENDIF()