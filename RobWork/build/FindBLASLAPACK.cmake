# - Find BLAS and LAPACK
# Finds the blas and lapack libraries
# if you want to specify the location of blas and lapack then set BLAS_ROOT or LAPACK_ROOT and
# If they are not set then they will try to be resolved automaticaly
#  
#  BLAS_LIBRARY_DIR   	- Hint as to where to find blas libraries (Not supported yet)
#  LAPACK_LIBRARY_DIR  	- Hint as to where to find lapack libraries (Not supported yet)
#  BLAS_LIBRARIES   	- List of blas libraries 
#  LAPACK_LIBRARIES	- List of lapack libraries
#  BLAS_FOUND       	- True if BLAS found.
#  LAPACK_FOUND		- True if LAPACK if found	

IF (BLAS_LIBRARY_DIR)
  # Already in cache, be silent.
  SET(LAPACK_FIND_QUIETLY TRUE)
ENDIF (BLAS_LIBRARY_DIR)

IF (LAPACK_LIBRARY_DIR)
  # Already in cache, be silent.
  SET(LAPACK_FIND_QUIETLY TRUE)
ENDIF (LAPACK_LIBRARY_DIR)


SET(BLAS_NAMES blas blas_win32)
FIND_LIBRARY(BLAS_LIBRARY NAMES ${BLAS_NAMES} PATHS ${BLAS_LIBRARY_DIR})

#MESSAGE("BLAS_LIBRARY = ${BLAS_LIBRARY}")

# Handle the QUIETLY and REQUIRED arguments and set PQP_FOUND to
# TRUE if all listed variables are TRUE.

# Handle the QUIETLY and REQUIRED arguments and set XERCESC_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  BLAS DEFAULT_MSG
  BLAS_LIBRARY 
)

IF(BLAS_FOUND)
  SET( BLAS_LIBRARIES ${BLAS_LIBRARY} )
ELSE(BLAS_FOUND)
  SET( BLAS_LIBRARIES  )
ENDIF(BLAS_FOUND)

SET(LAPACK_NAMES lapack lapack_win32) # add names here
FIND_LIBRARY(LAPACK_LIBRARY NAMES ${LAPACK_NAMES} PATHS ${LAPACK_LIBRARY_DIR})

# Handle the QUIETLY and REQUIRED arguments and set XERCESC_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  LAPACK DEFAULT_MSG
  LAPACK_LIBRARY 
)

IF(LAPACK_FOUND)
  SET( LAPACK_LIBRARIES ${LAPACK_LIBRARY} )
ELSE(LAPACK_FOUND)
  SET( LAPACK_LIBRARIES  )
ENDIF(LAPACK_FOUND)

#MESSAGE("LAPACK_LIBRARY = ${LAPACK_LIBRARY}")

MARK_AS_ADVANCED( LAPACK_LIBRARY LAPACK_INCLUDE_DIR BLAS_LIBRARY BLAS_INCLUDE_DIR )
