# Test CMake version
cmake_minimum_required(VERSION 2.6.0)

# The name of the project.
PROJECT(RobWork)
SET(ROBWORK_VERSION 0.3.1)
STRING( REGEX MATCHALL "[0-9]+" ROBWORK_VERSIONS ${ROBWORK_VERSION} )
LIST( GET ROBWORK_VERSIONS 0 ROBWORK_VERSION_MAJOR)
LIST( GET ROBWORK_VERSIONS 1 ROBWORK_VERSION_MINOR)
LIST( GET ROBWORK_VERSIONS 2 ROBWORK_VERSION_PATCH)

MESSAGE(STATUS "RobWork version ${ROBWORK_VERSION}")

IF (NOT RW_ROOT)
    SET(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF ()

SET(ROOT ${RW_ROOT})
# setup a standard project 
INCLUDE(${RW_ROOT}/build/ProjectSetup.cmake)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkConfig.cmake.in
  "${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

# set the postfix for debug configuration
SET(CMAKE_DEBUG_POSTFIX "_d")

# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
INCLUDE(${RW_ROOT}/build/RobWorkSetup.cmake)

LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )

# Subdirectories to process:
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

OPTION(RW_BUILD_TESTS "Set when you want to build the tests" ${RW_BUILD_TESTS})
IF( RW_BUILD_TESTS )
    MESSAGE(STATUS "Tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "Tests DISABLED!")
ENDIF()

OPTION(RW_BUILD_EXAMPLES "Set when you want to build the examples" ${RW_BUILD_EXAMPLES})
IF( RW_BUILD_EXAMPLES )
    MESSAGE(STATUS "Examples ENABLED!")
    ADD_SUBDIRECTORY(example)
ELSE ()
    MESSAGE(STATUS "Examples DISABLED!")
ENDIF()

# Try to find the current revision 
FIND_PACKAGE(Subversion)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${PROJECT_SOURCE_DIR} RobWork)
    MESSAGE("Current revision is ${Project_WC_REVISION}")
    Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork)
    MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Packaging
INCLUDE(build/packing.cmake)
