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
  SET(LIBF2C_FIND_QUIETLY TRUE)
ENDIF (LAPACK_LIBRARY_DIR)

# Find BLAS
SET(BLAS_NAMES blas)
IF(AMD64)
	LIST(APPEND BLAS_NAMES blas_win64)
ELSE()
	LIST(APPEND BLAS_NAMES blas_win32)
ENDIF()
FIND_LIBRARY(BLAS_LIBRARY NAMES ${BLAS_NAMES} PATHS ${BLAS_LIBRARY_DIR})

# Handle the QUIETLY and REQUIRED arguments and set BLAS_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  BLAS DEFAULT_MSG
  BLAS_LIBRARY 
)

# Store located BLAS libraries
IF(BLAS_FOUND)
  SET( BLAS_LIBRARIES ${BLAS_LIBRARY} )
ELSE(BLAS_FOUND)
  SET( BLAS_LIBRARIES  )
ENDIF(BLAS_FOUND)

# Find LAPACK
SET(LAPACK_NAMES lapack)
IF(AMD64)
  LIST(APPEND LAPACK_NAMES lapack_win64)
ELSE()
  LIST(APPEND LAPACK_NAMES lapack_win32)
ENDIF()
FIND_LIBRARY(LAPACK_LIBRARY NAMES ${LAPACK_NAMES} PATHS ${LAPACK_LIBRARY_DIR})

# Handle the QUIETLY and REQUIRED arguments and set LAPACK_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  LAPACK DEFAULT_MSG
  LAPACK_LIBRARY 
)

# Store located LAPACK libraries
IF(LAPACK_FOUND)
  SET( LAPACK_LIBRARIES ${LAPACK_LIBRARY})
ELSE()
  SET( LAPACK_LIBRARIES  )
ENDIF()

# Additional libraries needed by Windows
IF(WIN32)
  # Find F2C
  SET(LIBF2C_NAMES libf2c)
  IF(AMD64)
    LIST(APPEND LIBF2C_NAMES libf2c_win64)
  ELSE()
    LIST(APPEND LIBF2C_NAMES libf2c_win32)
  ENDIF()
  FIND_LIBRARY(LIBF2C_LIBRARY NAMES ${LIBF2C_NAMES} PATHS ${BLAS_LIBRARY_DIR} ${LAPACK_LIBRARY_DIR})

  # Handle the QUIETLY and REQUIRED arguments and set LIBF2C_FOUND to
  # TRUE if all listed variables are TRUE.
  INCLUDE(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    LIBF2C DEFAULT_MSG
    LIBF2C_LIBRARY 
  )
  
  IF(LIBF2C_FOUND)
    LIST(APPEND LAPACK_LIBRARIES ${LIBF2C_LIBRARY})
  ENDIF()
  
  # Find RUNTMCHK
  SET(RUNTMCHK_NAMES runtmchk)
  IF(AMD64)
    LIST(APPEND RUNTMCHK_NAMES runtmchk_win64)
  ELSE()
    LIST(APPEND RUNTMCHK_NAMES runtmchk_win32)
  ENDIF()
  FIND_LIBRARY(RUNTMCHK_LIBRARY NAMES ${RUNTMCHK_NAMES} PATHS ${BLAS_LIBRARY_DIR} ${LAPACK_LIBRARY_DIR})

  # Handle the QUIETLY and REQUIRED arguments and set RUNTMCHK_FOUND to
  # TRUE if all listed variables are TRUE.
  INCLUDE(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    RUNTMCHK DEFAULT_MSG
    RUNTMCHK_LIBRARY 
  )    

  IF(RUNTMCHK_FOUND)
    LIST(APPEND LAPACK_LIBRARIES ${RUNTMCHK_LIBRARY})
  ENDIF()
ENDIF(WIN32)

MARK_AS_ADVANCED( LAPACK_LIBRARY LAPACK_INCLUDE_DIR BLAS_LIBRARY BLAS_INCLUDE_DIR )
