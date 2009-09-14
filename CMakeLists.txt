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

# setup the install dirs TODO should be cached to allow the use to change it 
SET(RW_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
SET(RW_LIB_INSTALL_DIR "lib/${CMAKE_BUILD_TYPE}/")
SET(RW_EXT_INSTALL_DIR "ext/")
SET(RW_HPP_INSTALL_DIR "include/")
SET(RW_RUNTIME runtime)
SET(RW_DEVEL development)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkConfig.cmake.in
  "${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
INCLUDE(${RW_ROOT}/build/RobWorkSetup.cmake)

LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
LINK_DIRECTORIES( ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )

# Subdirectories to process:
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

OPTION(RW_BUILD_TESTS "Set when you want to build the tests" ${RW_BUILD_TESTS})
IF( RW_BUILD_TESTS )
    MESSAGE(STATUS "RobWork tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWork tests DISABLED!")
ENDIF()

# Try to find the current revision 
FIND_PACKAGE(Subversion)
IF(Subversion_FOUND)
    #Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE("Current revision is ${RobWork_WC_REVISION}")
    #Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork) # not foundd in linux
    #MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# Packaging
INCLUDE(build/packing.cmake)

# install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "./")
INSTALL(FILES build/FindXercesC.cmake DESTINATION "config/")
INSTALL(FILES "build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake" "build/FindRobWork.cmake" DESTINATION "config/")
INSTALL(FILES ${RW_ROOT}/src/RobWorkConfig.hpp DESTINATION "include/")

