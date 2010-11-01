#
# This is a default project setup. It enables multiple build trees for 
# multiple configuration eg. CMAKE_BUILD_TYPE 
# 
# input:
# ROOT : root of the project folder
#


# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Enable new linker path policy.
IF (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
ENDIF ()

# A shorter alias for this directory.
IF (NOT ROOT)
    MESSAGE(FATAL_ERROR "ROOT must be defined before calling ProjectSetup")
ENDIF ()

# Specify wether to default compile in Release, Debug, MinSizeRel, RelWithDebInfo mode
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
ENDIF ()
MESSAGE(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")

# Load the optional Default.cmake file.
INCLUDE(${ROOT}/config.cmake OPTIONAL)
IF (NOT EXISTS ${ROOT}/config.cmake)
  # Setup the default settings in case no RobWork.cmake exist.
  INCLUDE(${ROOT}/config.cmake.template)
  MESSAGE(STATUS "No config.cmake file loaded, using default settings from config.cmake.template")
ENDIF ()

# Output goes to bin/<CONFIG> and libs/<CONFIG> unless specified otherwise by the user.
IF (DEFINED MSVC)
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE ()
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
    SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF ()
