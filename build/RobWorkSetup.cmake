
#
# Allow the syntax else (), endif (), etc.
#
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Check if RW_ROOT path are setup correctly
FIND_FILE(RW_ROOT_PATH_TEST RobWork.cmake.template ${RW_ROOT} NO_DEFAULT_PATH)
IF(NOT RW_ROOT_PATH_TEST)
 MESSAGE(SEND_ERROR "Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()

#
# Load the optional RobWork.cmake file.
# 
INCLUDE(${RW_ROOT}/RobWork.cmake OPTIONAL)
IF (NOT EXISTS ${RW_ROOT}/RobWork.cmake)
  # Setup the default settings in case no RobWork.cmake exist. Check out
  # RobWork.cmake.template for description
  INCLUDE(${RW_ROOT}/RobWork.cmake.template)
ENDIF ()

# Setup opengl libraries
IF (${UseOpenGL})
	MESSAGE(STATUS "-- Use OpenGL" "")
    INCLUDE(FindOpenGL)	
ENDIF ()

INCLUDE(${RW_ROOT}/build/include.cmake)
INCLUDE(${RW_ROOT}/build/tests.cmake)
INCLUDE(${RW_ROOT}/build/link.cmake)

# Setup the LibraryList 
SET(LibraryList
  ${SANDBOX_LIB}
  rw_algorithms
  rw_pathplanners
  rw_pathoptimization
  rw_proximitystrategies
  rw_drawable
  rw_simulation
  rw
  ${UblasLibraries}
  ${CollisionDetectionLibraries}
  ${OPENGL_LIBRARIES}
  ${LibraryList}
)

