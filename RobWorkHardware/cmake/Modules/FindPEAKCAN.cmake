# CMake module to search for PEAK CAN library
#
# If it's found it sets PEAKCAN_FOUND to TRUE
# and following variables are set:
#    PEAKCAN_INCLUDE_DIR
#    PEAKCAN_LIBRARY
#
#    PEAKCAN_FOUND

IF( UNIX AND NOT CYGWIN )
    FIND_PATH(PEAKCAN_INCLUDE_DIR libpcan.h
              PATHS 
              /usr/include 
              /include
    )
ELSEIF( CYGWIN OR WIN32) 
    FIND_PATH(PEAKCAN_INCLUDE_DIR Pcan_usb.h
       # TODO paths?
       PATHS
       /usr/include
       /include
    )
ENDIF()

FIND_LIBRARY(PEAKCAN_LIBRARIES
             NAMES "pcan" 
             PATHS "/usr/lib")

IF (PEAKCAN_INCLUDE_DIR AND PEAKCAN_LIBRARIES)
    #MESSAGE(STATUS "SDH: Found!")
    SET (PEAKCAN_FOUND 1)
ELSE()
    #MESSAGE(STATUS "SDH: Not found!")
ENDIF()
