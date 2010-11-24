#####################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source 
# files and when configuring your project.  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project. (EDIT THIS)
PROJECT(ConsoleApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
IF(WIN32)
  # On Windows platforms, the RW/RWS installers create the necessary environment variables
  IF(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
    SET(RW_ROOT "$ENV{RW_ROOT}")
    SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")
  ELSE()
    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
  ENDIF()
ELSEIF(UNIX)
  # On Ubuntu platforms, the Debian packages install into /usr/local or /usr
  IF(EXISTS "/usr/local/RobWork" AND EXISTS "/usr/local/RobWorkStudio")
    SET(RW_ROOT "/usr/local/RobWork")
    SET(RWSTUDIO_ROOT "/usr/local/RobWorkStudio")
  ELSEIF(EXISTS "/usr/RobWork" AND EXISTS "/usr/RobWorkStudio")
    SET(RW_ROOT "/usr/RobWork")
    SET(RWSTUDIO_ROOT "/usr/RobWorkStudio")
  ELSE()
    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
  ENDIF()
ELSE()
  SET(RW_ROOT "${ROOT}/../../../RobWork")
  SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
ENDIF()

# set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
SET(ROBWORK_ROOT ${RW_ROOT})
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

# if you have additional libraries or include dirs then add them here
SET(USER_LIBRARIES )
INCLUDE_DIRECTORIES(${ROOT}/src/)
LINK_DIRECTORIES( )

# And now we add any targets that we want
add_executable(SampleTest SampleTest.cpp)
target_link_libraries(SampleTest ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

