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

#MESSAGE("XERCSE_INCLUDE_DIR = ${XERCESC_INCLUDE_DIR}")
#MESSAGE("XERCSE_INCLUDE_DIR = ${XERCESC_LIB_DIR}")

FIND_PATH(XERCESC_INCLUDE_DIR_TMP 
    xercesc 
    HINTS ${XERCESC_INCLUDE_DIR} /usr/include
    PATHS ${XERCESC_INCLUDE_DIR} /usr/include
)

SET(XERCESC_NAMES_STATIC xerces-c.a
						 libxerces-c.a
						 xerces-c_static
						 xerces-c_static_2
						 xerces-c_static_2_8
						 xerces-c_static_3
						 xerces-c_static_3_1
)
SET(XERCESC_NAMES_SHARED xerces-c
						 xerces-c-2
						 xerces-c-2.8
						 xerces-c-3
						 xerces-c-3.1
						 xerces-c_2
						 xerces-c_2_8
						 xerces-c_3
						 xerces-c_3_1
)
FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES_STATIC} ${XERCESC_NAMES_SHARED}
							 PATHS ${XERCESC_LIB_DIR} )
# Check if we found the static version or not
SET(XERCES_USE_STATIC_LIBS ON)
GET_FILENAME_COMPONENT(XERCESC_LIBRARY_WE ${XERCESC_LIBRARY} NAME_WE)
LIST(FIND XERCESC_NAMES_STATIC ${XERCESC_LIBRARY_WE} STAT)
IF(${STAT} EQUAL -1)
	SET(XERCES_USE_STATIC_LIBS OFF)
ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set XERCESC_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  XERCESC DEFAULT_MSG
  XERCESC_LIBRARY XERCESC_INCLUDE_DIR_TMP
)

IF(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES ${XERCESC_LIBRARY} )
  SET( XERCESC_INCLUDE_DIR ${XERCESC_INCLUDE_DIR_TMP} )
ELSE(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES )
  SET( XERCESC_INCLUDE_DIR )
ENDIF(XERCESC_FOUND)

MARK_AS_ADVANCED( XERCESC_LIBRARY XERCESC_INCLUDE_DIR )
