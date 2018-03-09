# - Find Xerces-C
# Find the Xerces-C includes and library
# if you want to specify the location of xerces then set XERCESC_ROOT or XERCESC_INCLUDE_DIR and
# XERCESC_LIB_DIR. If they are not set then they will try to be resolved automaticaly
#  
#  XERCESC_ROOT        - Hint as to where to find include and lib dirs.
#  XERCESC_INCLUDE_DIR - Where to find xercesc include sub-directory.
#  XERCESC_LIB_DIR     - Where to find xercesc lib sub-directory.
#  XERCESC_LIBRARIES   - List of libraries when using Xerces-C.
#  XERCESC_FOUND       - True if Xerces-C found.

IF(NOT DEFINED XERCESC_ROOT AND DEFINED ENV{XERCESC_ROOT})
	SET(XERCESC_ROOT $ENV{XERCESC_ROOT})
ENDIF()

IF (XERCESC_INCLUDE_DIR)
  # Already in cache, be silent.
  SET(XERCESC_FIND_QUIETLY TRUE)
ENDIF (XERCESC_INCLUDE_DIR)

FIND_PATH(XERCESC_INCLUDE_DIR_TMP 
    xercesc 
    HINTS ${XERCESC_INCLUDE_DIR} ENV XERCESC_INCLUDE_DIR
    PATHS ${XERCESC_INCLUDE_DIR} ENV XERCESC_INCLUDE_DIR
    NO_DEFAULT_PATH
)
FIND_PATH(XERCESC_INCLUDE_DIR_TMP 
    xercesc 
    HINTS ${XERCESC_ROOT}/include
    PATHS ${XERCESC_ROOT}/include
    NO_DEFAULT_PATH
)
FIND_PATH(XERCESC_INCLUDE_DIR_TMP 
    xercesc
    PATHS /usr/include
)

SET(XERCESC_NAMES_STATIC_DEBUG 
  			   xerces-cD.a
               libxerces-cD.a
               xerces-c_staticD
               xerces-c_static_2D
               xerces-c_static_2_8D
               xerces-c_static_3D
               xerces-c_static_3_1D
               xerces-c_static_3_2D
)
SET(XERCESC_NAMES_SHARED_DEBUG 
			   xerces-cD
               xerces-c-2D
               xerces-c-2.8D
               xerces-c-3D
               xerces-c-3.1D
               xerces-c-3.2D
               xerces-c_2D
               xerces-c_2_8D
               xerces-c_3D
               xerces-c_3_1D
               xerces-c_3_2D
)

  SET(XERCESC_NAMES_STATIC_RELEASE
	       xerces-c.a
               libxerces-c.a
               xerces-c_static
               xerces-c_static_2
               xerces-c_static_2_8
               xerces-c_static_3
               xerces-c_static_3_1
               xerces-c_static_3_2
  )
  SET(XERCESC_NAMES_SHARED_RELEASE
	       xerces-c
               xerces-c-2
               xerces-c-2.8
               xerces-c-3
               xerces-c-3.1
               xerces-c-3.2
               xerces-c_2
               xerces-c_2_8
               xerces-c_3
               xerces-c_3_1
               xerces-c_3_2
               xerces-c_3D
  )

IF("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  SET(XERCESC_NAMES_SHARED ${XERCESC_NAMES_SHARED_DEBUG} )
  SET(XERCESC_NAMES_STATIC ${XERCESC_NAMES_STATIC_DEBUG} )
ELSE()
  SET(XERCESC_NAMES_SHARED ${XERCESC_NAMES_SHARED_RELEASE} )
  SET(XERCESC_NAMES_STATIC ${XERCESC_NAMES_STATIC_RELEASE} )
ENDIF()

IF(UNIX)
	SET(XERCESC_NAMES_LINUX "${XERCESC_NAMES_SHARED};${XERCESC_NAMES_SHARED_RELEASE}") 
	#MESSAGE("${XERCESC_NAMES_LINUX}")
	FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES_LINUX}
								 PATHS ${XERCESC_LIB_DIR} ENV XERCESC_LIB_DIR)
	FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES_LINUX}
								 PATHS ${XERCESC_ROOT}/lib)
	SET(XERCES_USE_STATIC_LIBS OFF)
ELSE()
	FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES_STATIC} ${XERCESC_NAMES_SHARED}
								 PATHS ${XERCESC_LIB_DIR} ENV XERCESC_LIB_DIR)
	FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES_STATIC} ${XERCESC_NAMES_SHARED}
								 PATHS ${XERCESC_ROOT}/lib)

	# Check if we found the static version or not
	SET(XERCES_USE_STATIC_LIBS ON)
	GET_FILENAME_COMPONENT(XERCESC_LIBRARY_WE "${XERCESC_LIBRARY}" NAME_WE)
	LIST(FIND XERCESC_NAMES_SHARED ${XERCESC_LIBRARY_WE} STAT)
	IF(${STAT} GREATER -1)
		SET(XERCES_USE_STATIC_LIBS OFF)
	ENDIF()
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
