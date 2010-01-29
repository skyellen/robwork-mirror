

FIND_FILE(NTCAN "ntcan.h" ${ESDCAN_INCLUDE_DIR})
IF(NOT NTCAN)
 MESSAGE(STATUS "ESDCAN disabled! ESDCAN not Found!")
ELSE ()
 MESSAGE(STATUS "ESDCAN enabled! ESDCAN Found!")
 
 INCLUDE_DIRECTORIES( ${ESDCAN_INCLUDE_DIR} )
 SET(CANFiles ./IEICAN02/icanapi.cpp
              ./IEICAN02/IEICANPort.cpp
              ./ESDCAN/ESDCANPort.cpp)
ENDIF ()