SET(SUBSYS_NAME motomanIA20 )
SET(SUBSYS_DESC "Library for controlling the motomanIA20 robot" )
SET(SUBSYS_DEPS rw )



set(DEFAULT TRUE)
set(REASON )
IF( DEFINED WIN32)
    set(DEFAULT false)
    set(REASON "Does not support windows!")
endif()
 
SET(build TRUE)
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
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