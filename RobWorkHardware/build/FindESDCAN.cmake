

FIND_PATH(ESDCAN_INCLUDE_DIR ntcan.h
	"$ENV{ProgramFiles}/ESD/CAN/SDK/include"
)
IF(NOT ESDCAN_INCLUDE_DIR)
 MESSAGE(STATUS "ESDCAN disabled! ESDCAN not Found!")
ELSE ()
 SET(ESDCAN_FOUND 1)
 MESSAGE(STATUS "ESDCAN enabled! ESDCAN Found!")
 
 #INCLUDE_DIRECTORIES( ${ESDCAN_INCLUDE_DIR} )
 #SET(CANFiles ./IEICAN02/icanapi.cpp
 #            ./IEICAN02/IEICANPort.cpp
 #             ./ESDCAN/ESDCANPort.cpp)
ENDIF ()