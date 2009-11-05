# -*- cmake -*-
# this should be configured before running this script

LINK_DIRECTORIES(${RW_ARCHIVE_OUT_DIR} ${RW_LIBRARY_OUT_DIR})
LIST(APPEND CMAKE_LIBRARY_PATH ${RW_LIBRARY_OUT_DIR})

# All mandatory libraries for linking with rw:
if (DEFINED MINGW)
  #set(RW_UBLAS_LIBRARY_NAMES lapack blas g2c)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED MSVC)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED UNIX)
  set(RW_UBLAS_LIBRARY_NAMES lapack)
endif ()

# Libraries for programs using rw.
set(RW_LIBRARY_LIST "rw" ${RW_UBLAS_LIBRARY_NAMES})

# Opengl
IF (NOT OPENGL_FOUND)
    MESSAGE("OpenGL not found! Libraries that depent on OpenGL will not be compiles!")
ENDIF ()

# Libraries for programs using rw_drawable.
set(RW_DRAWABLE_LIBRARY_LIST
  "rw_drawable"
  ${RW_LIBRARY_LIST}
  ${OPENGL_LIBRARIES}
  )


# Libraries for programs using rw_lua.
set(RW_LUA_LIBRARY_LIST
  "rw_lua"
  ${RW_LIBRARY_LIST}
  "tolualib"
  "lualib"
  )

# Libraries for programs using rw_pathplanners.
set(RW_PATHPLANNERS_LIBRARY_LIST
  "rw_pathplanners"
  ${RW_LIBRARY_LIST}
  )

# add the enabled collision detection libraries
SET(RW_COLLISION_DETECTION_LIBS)
if (RW_HAVE_PQP)
  list(APPEND RW_COLLISION_DETECTION_LIBS ${PQP_LIB})
endif ()

if (RW_HAVE_YAOBI)
  list(APPEND RW_COLLISION_DETECTION_LIBS ${YAOBI_LIB})
endif ()

# Libraries for programs using rw_proximitystrategies.
set(RW_PROXIMITYSTRATEGIES_LIBRARY_LIST
    "rw_proximitystrategies"
    ${RW_LIBRARY_LIST}
    ${RW_COLLISION_DETECTION_LIBS}
  )

# etc...

IF (RW_BUILD_SANDBOX)
    MESSAGE(STATUS "RobWork Sandbox ENABLED!")
    SET(SANDBOX_LIB "rw_sandbox")
ELSE ()
    MESSAGE(STATUS "RobWork Sandbox DISABLED!")    
ENDIF ()

# We should use a more standard technique for the packaging of libraries and
# their dependencies (probably there are conventions for this in CMake already).
