# CMake module to search for TrakStar library
#
# If it's found it sets TRAKSTAR_FOUND to TRUE
# and following variables are set:
#    TRAKSTAR_INCLUDE_DIR
#    TRAKSTAR_LIBRARIES
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
             PATHS 
             "/usr/lib"
             "/usr/lib64"
             /opt/3DGuidance.Rev.E.64/3DGuidanceAPI)


FIND_LIBRARY(TRAKSTAR_LIBRARIES
             NAMES "ATC3DGlib64.so" 
             PATHS 
             "/usr/lib"
             "/usr/lib64"
             "/opt/3DGuidance.Rev.E.64/3DGuidanceAPI")



#MESSAGE("${TRAKSTAR_LIBRARIES}")
#MESSAGE("${TRAKSTAR_INCLUDE_DIR}")

IF (TRAKSTAR_INCLUDE_DIR AND TRAKSTAR_LIBRARIES)
    SET (TRAKSTAR_FOUND 1)
    #GET_TARGET_PROPERTY(libATC3D_location libATC3D LOCATION) # deprecated (fails on non-existing targets)
    IF(TARGET libATC3D)
        # target allready exists
    ELSE()
        ADD_LIBRARY(libATC3D SHARED IMPORTED)
        SET_PROPERTY(TARGET libATC3D PROPERTY IMPORTED_LOCATION ${TRAKSTAR_LIBRARIES})
    ENDIF()
    SET(TRAKSTAR_LIBRARIES libATC3D) 
ELSE()

ENDIF()
