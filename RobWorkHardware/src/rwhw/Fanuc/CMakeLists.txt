SET(SUBSYS_NAME fanucdriver)
SET(SUBSYS_DESC "Library of drivers for controlling fanuc robot" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 

IF(COMPONENT_${SUBSYS_NAME}_ENABLE AND NOT DEFINED WIN32)
  SET(BUILD_${SUBSYS_NAME} FALSE)
  set(REASON "WIN32 only")
ENDIF()

RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )
 
IF( build AND DEFINED WIN32)
    #MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    

    ADD_LIBRARY(
      rwhw_fanucdevice
      ./FanucDriver.cpp
      ./FanucVirtual.cpp
      ./VelRampProfile.cpp
    )
    SET( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_fanucdevice;")
ENDIF()

