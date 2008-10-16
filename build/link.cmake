# -*- cmake -*-
message("LibraryOutput = " ${LIBRARY_OUTPUT_PATH})
set(d ${LIBRARY_OUTPUT_PATH})
link_directories(${d})
list(APPEND CMAKE_LIBRARY_PATH ${d})

# All mandatory libraries for linking with rw:
if (DEFINED MINGW)
  set(RW_UBLAS_LIBRARY_NAMES lapack blas g2c)
elseif (DEFINED MSVC)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED UNIX)
  set(RW_UBLAS_LIBRARY_NAMES lapack)
endif ()

# Find pqp, and yaobi in case the user has installed these already, or
# use their raw names as defaults.
find_library(PQP_LIB pqp)
if (NOT PQP_LIB)
  message("Found PQP Collision Detector")
  set(PQP_LIB pqp)
endif ()

message( "Find collision libs")

find_library(YAOBI_LIB yaobi)
if (NOT YAOBI_LIB)
  message("Found Yaobi Collision Detector")
  set(YAOBI_LIB yaobi)
endif ()

# Libraries for programs using rw.
set(RW_LIBRARY_LIST
  rw
  ${RW_UBLAS_LIBRARY_NAMES}
  )

# Libraries for programs using rw_drawable.
include(FindOpenGL)
set(RW_DRAWABLE_LIBRARY_LIST
  rw_drawable
  ${RW_LIBRARY_LIST}
  ${OPENGL_LIBRARIES}
  )

# Libraries for programs using rw_lua.
include(FindOpenGL)
set(RW_LUA_LIBRARY_LIST
  rw_lua
  ${RW_LIBRARY_LIST}
  tolualib
  lualib
  )

# Libraries for programs using rw_pathplanners.
include(FindOpenGL)
set(RW_PATHPLANNERS_LIBRARY_LIST
  rw_pathplanners
  ${RW_LIBRARY_LIST}
  )

set(CollisionDetectionLibraries)
if (RW_HAVE_PQP)
  list(APPEND CollisionDetectionLibraries pqp)
  message("Have PQP")
else()
	message("No PQP")
endif ()
if (RW_HAVE_YAOBI)
  list(APPEND CollisionDetectionLibraries yaobi)
	message("Have Yaobi")
else()
	message("No Yaobi")
endif ()

# Libraries for programs using rw_proximitystrategies.
set(RW_PROXIMITYSTRATEGIES_LIBRARY_LIST
  rw_proximitystrategies
  ${RW_LIBRARY_LIST}
  ${CollisionDetectionLibraries}
  )

# etc...

# We should use a more standard technique for the packaging of libraries and
# their dependencies (probably there are conventions for this in CMake already).
