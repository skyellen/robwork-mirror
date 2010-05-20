# CMake module to search for ESD CAN library
#
# If it's found it sets ESDCAN_FOUND to TRUE
# and following variables are set:
#    ESDCAN_INCLUDE_DIR
#    ESDCAN_LIBRARY

FIND_PATH(ESDCAN_INCLUDE_DIR ntcan.h
	"$ENV{ProgramFiles}/ESD/CAN/SDK/include"
	/usr/local/esd/include
	/usr/local/include
	/usr/include
    /include
)
IF(NOT ESDCAN_INCLUDE_DIR)
 MESSAGE(STATUS "ESDCAN disabled! ESDCAN not Found!")
ELSE ()
 SET(ESDCAN_FOUND 1)
 
 #FIND_LIBRARY(ESDCAN_LIBRARY NAMES ntcan)
 FIND_LIBRARY(ESDCAN_LIBRARY NAMES ntcan PATHS . /usr/local/esd/lib64 "$ENV{ProgramFiles}/ESD/CAN/SDK/lib")
 
 
 MESSAGE(STATUS "ESDCAN enabled! ESDCAN Found! ${ESDCAN_INCLUDE_DIR} ${ESDCAN_LIBRARY}")
 
 #INCLUDE_DIRECTORIES( ${ESDCAN_INCLUDE_DIR} )
 #SET(CANFiles ./IEICAN02/icanapi.cpp
 #            ./IEICAN02/IEICANPort.cpp
 #             ./ESDCAN/ESDCANPort.cpp)
ENDIF ()