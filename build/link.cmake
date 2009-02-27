# -*- cmake -*-
# this should be configured before running this script

LINK_DIRECTORIES(${RW_ARCHIVE_OUT_DIR} ${RW_LIBRARY_OUT_DIR})
LIST(APPEND CMAKE_LIBRARY_PATH ${RW_LIBRARY_OUT_DIR})

# All mandatory libraries for linking with rw:
if (DEFINED MINGW)
  set(RW_UBLAS_LIBRARY_NAMES lapack blas g2c)
elseif (DEFINED MSVC)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED UNIX)
  set(RW_UBLAS_LIBRARY_NAMES lapack)
endif ()

IF(XERCESC_FOUND)
    SET(XERCES_LIB XERCESC_LIBRARIES)
ENDIF ()

# Find pqp, and yaobi in case the user has installed these already, or
# use their raw names as defaults.
if (RW_HAVE_PQP)
	find_library(PQP_LIB "pqp${RW_POSTFIX}" ${RW_ARCHIVE_OUT_DIR})
	if (NOT PQP_LIB)
	  set(PQP_LIB "pqp${RW_POSTFIX}")
	  message(STATUS "WARNING: Could not find PQP library. Using default name '${PQP_LIB}'")
	endif ()
endif()

if (RW_HAVE_YAOBI)
	find_library(YAOBI_LIB "yaobi${RW_POSTFIX}" ${RW_ARCHIVE_OUT_DIR})
	if (NOT YAOBI_LIB)
	  set(YAOBI_LIB "yaobi${RW_POSTFIX}")
	  message(STATUS "WARNING: Could not find yaobi library. Using default name ${YAOBI_LIB}")
	endif ()
endif()

# Libraries for programs using rw.
set(RW_LIBRARY_LIST
  "rw${RW_POSTFIX}"
  ${RW_UBLAS_LIBRARY_NAMES}
  )

# Opengl
IF (NOT OPENGL_FOUND)
    MESSAGE("OpenGL not found! Libraries that depent on OpenGL will not be compiles!")
ENDIF ()

# Libraries for programs using rw_drawable.
set(RW_DRAWABLE_LIBRARY_LIST
  "rw_drawable${RW_POSTFIX}"
  ${RW_LIBRARY_LIST}
  ${OPENGL_LIBRARIES}
  )


# Libraries for programs using rw_lua.
set(RW_LUA_LIBRARY_LIST
  "rw_lua${RW_POSTFIX}"
  ${RW_LIBRARY_LIST}
  "tolualib${RW_POSTFIX}"
  "lualib${RW_POSTFIX}"
  )

# Libraries for programs using rw_pathplanners.
set(RW_PATHPLANNERS_LIBRARY_LIST
  "rw_pathplanners${RW_POSTFIX}"
  ${RW_LIBRARY_LIST}
  )

SET(RW_COLLISION_DETECTION_LIBS)
if (RW_HAVE_PQP)
  list(APPEND RW_COLLISION_DETECTION_LIBS ${PQP_LIB})
endif ()

if (RW_HAVE_YAOBI)
  list(APPEND RW_COLLISION_DETECTION_LIBS ${YAOBI_LIB})
endif ()

# Libraries for programs using rw_proximitystrategies.
set(RW_PROXIMITYSTRATEGIES_LIBRARY_LIST
    "rw_proximitystrategies${RW_POSTFIX}"
    ${RW_LIBRARY_LIST}
    ${RW_COLLISION_DETECTION_LIBS}
  )

# etc...

IF (DEFINED COMPILE_SANDBOX)
    MESSAGE(STATUS "Sandbox ENABLED!")
    SET(SANDBOX_LIB "rw_sandbox${RW_POSTFIX}")
ELSE ()
    MESSAGE(STATUS "Sandbox DISABLED!")    
ENDIF ()

# We should use a more standard technique for the packaging of libraries and
# their dependencies (probably there are conventions for this in CMake already).
