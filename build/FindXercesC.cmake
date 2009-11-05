# - Find Xerces-C
# Find the Xerces-C includes and library
# if you want to specify the location of xerces then set XERCESC_ROOT or XERCESC_INCLUDE_DIR and
# XERCES_LIB_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  XERCESC_ROOT        - Hint as to where to find include and lib dirs (Not supported yet)
#  XERCESC_INCLUDE_DIR - Where to find xercesc include sub-directory.
#  XERCESC_LIB_DIR     - Where to find xercesc lib sub-directory.
#  XERCESC_LIBRARIES   - List of libraries when using Xerces-C.
#  XERCESC_FOUND       - True if Xerces-C found.

IF (XERCESC_INCLUDE_DIR)
  # Already in cache, be silent.
  SET(XERCESC_FIND_QUIETLY TRUE)
ENDIF (XERCESC_INCLUDE_DIR)

FIND_PATH(XERCESC_INCLUDE_DIR xercesc)

SET(XERCESC_NAMES xerces-c xerces-c_2 xerces-c_3)
FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES} PATHS ${XERCESC_LIB_DIR} )



# Handle the QUIETLY and REQUIRED arguments and set XERCESC_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  XERCESC DEFAULT_MSG
  XERCESC_LIBRARY XERCESC_INCLUDE_DIR
)

IF(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES ${XERCESC_LIBRARY} )
ELSE(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES )
ENDIF(XERCESC_FOUND)

MARK_AS_ADVANCED( XERCESC_LIBRARY XERCESC_INCLUDE_DIR )
