# CMake module to search for PEAK CAN library
#
# If it's found it sets PEAKCAN_FOUND to TRUE
# and following variables are set:
#    TRAKSTAR_INCLUDE_DIR
#    TRAKSTAR_LIBRARY
#
#    TRAKSTAR_FOUND

IF( UNIX AND NOT CYGWIN )
    FIND_PATH(TRAKSTAR_INCLUDE_DIR ATC3DG.h
       # TODO paths?
       /usr/include
       /include
       /opt/3DGuidance.Rev.E.64/3DGuidanceAPI
    )
ELSEIF( CYGWIN OR WIN32) 
    FIND_PATH(TRAKSTAR_INCLUDE_DIR ATC3DG.h
       # TODO paths?
       /usr/include
       /include
    )
ENDIF()

FIND_PATH(TRAKSTAR_LIBRARY_DIR
             ATC3DGlib64.so 
             PATHS "/usr/lib"
             /opt/3DGuidance.Rev.E.64/3DGuidanceAPI)


FIND_LIBRARY(TRAKSTAR_LIBRARIES
             NAMES "ATC3DGlib64.so" 
             PATHS "/usr/lib"
             /opt/3DGuidance.Rev.E.64/3DGuidanceAPI)



#MESSAGE("${TRAKSTAR_LIBRARIES}")
#MESSAGE("${TRAKSTAR_INCLUDE_DIR}")

IF (TRAKSTAR_INCLUDE_DIR AND TRAKSTAR_LIBRARIES)
    SET (TRAKSTAR_FOUND 1)
ELSE()

ENDIF()
