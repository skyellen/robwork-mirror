

FIND_FILE(IEICAN "icanapi.h" ${IEICAN_INCLUDE_DIR})
IF(NOT NTCAN)
 MESSAGE(STATUS "IEICAN disabled! ESDCAN not Found!")
ELSE ()
 MESSAGE(STATUS "IEICAN enabled! ESDCAN Found!")
 INCLUDE_DIRECTORIES( ${ESDCAN_INCLUDE_DIR} )
 SET(CANFiles ./IEICAN02/icanapi.cpp
              ./IEICAN02/IEICANPort.cpp
              ./ESDCAN/ESDCANPort.cpp)
ENDIF ()