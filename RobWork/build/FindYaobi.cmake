# - Find YAOBI
# Find the YAOBI includes and library
# if you want to specify the location of xerces then set YAOBI_ROOT or YAOBI_INCLUDE_DIR and
# YAOBI_LIB_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  YAOBI_ROOT        - Hint as to where to find include and lib dirs (Not supported yet)
#  YAOBI_INCLUDE_DIR - Where to find xercesc include sub-directory.
#  YAOBI_LIBRARY_DIR     - Where to find xercesc lib sub-directory.
#  YAOBI_LIBRARIES   - List of libraries when using Xerces-C.
#  YAOBI_FOUND       - True if Xerces-C found.

IF (YAOBI_INCLUDE_DIR)
    # Already in cache, be silent.
    SET(YAOBI_FIND_QUIETLY TRUE)
ENDIF(YAOBI_INCLUDE_DIR)

FIND_PATH(YAOBI_INCLUDE_DIR_TMP "yaobi.h" PATHS ${YAOBI_INCLUDE_DIR})

SET(YAOBI_NAMES yaobi)
FIND_LIBRARY(YAOBI_LIBRARY NAMES ${YAOBI_NAMES} PATHS ${YAOBI_LIBRARY_DIR} )
#MESSAGE(" ${YAOBI_LIBRARY} ")
#MESSAGE(" ${YAOBI_LIB_DIR} ")
# Handle the QUIETLY and REQUIRED arguments and set YAOBI_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  YAOBI DEFAULT_MSG
  YAOBI_LIBRARY 
  YAOBI_INCLUDE_DIR_TMP
)

IF(YAOBI_FOUND)
  SET( YAOBI_LIBRARIES ${YAOBI_LIBRARY} )
  SET( YAOBI_INCLUDE_DIR ${YAOBI_INCLUDE_DIR_TMP} )
ELSE(YAOBI_FOUND)
  SET( YAOBI_LIBRARIES )
  SET( YAOBI_INCLUDE_DIR )
ENDIF(YAOBI_FOUND)

MARK_AS_ADVANCED( YAOBI_LIBRARY YAOBI_INCLUDE_DIR )
