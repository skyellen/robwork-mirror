cmake_minimum_required(VERSION 2.6.0)

IF( CMAKE_VERSION VERSION_LESS 2.6.4 ) 

ELSE( CMAKE_VERSION VERSION_LESS 2.6.4)
    CMAKE_POLICY(SET CMP0011 NEW)
ENDIF( CMAKE_VERSION VERSION_LESS 2.6.4)

PROJECT(RobWorkSim)
SET(ROBWORKSIM_VERSION 0.5.1)
STRING( REGEX MATCHALL "[0-9]+" ROBWORKSIM_VERSIONS ${ROBWORKSIM_VERSION} )
LIST( GET ROBWORKSIM_VERSIONS 0 ROBWORKSIM_VERSION_MAJOR)
LIST( GET ROBWORKSIM_VERSIONS 1 ROBWORKSIM_VERSION_MINOR)
LIST( GET ROBWORKSIM_VERSIONS 2 ROBWORKSIM_VERSION_PATCH)

MESSAGE(STATUS "ROBWORKSIM version ${ROBWORKSIM_VERSION}")

# Used to resolve absolute path names
# Used to resolve absolute path names
IF (NOT RWSIM_ROOT)    
    SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    SET(RWSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
ELSE ()
    SET(ROOT ${RWSIM_ROOT})
ENDIF ()


#Include default settings for constructing a robwork dependent project
INCLUDE(${RWSIM_ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it
IF(UNIX)
	SET(RWSIM_INSTALL_DIR "RobWorkSim/")
ELSE(UNIX)
	SET(RWSIM_INSTALL_DIR "./")
ENDIF(UNIX)
SET(RWSIM_BIN_INSTALL_DIR "${RWSIM_INSTALL_DIR}/bin/")
SET(RWSIM_LIB_INSTALL_DIR "${RWSIM_INSTALL_DIR}/libs/")
SET(RWSIM_EXT_INSTALL_DIR "${RWSIM_INSTALL_DIR}/ext/")
SET(RWSIM_HPP_INSTALL_DIR "${RWSIM_INSTALL_DIR}/src/")
SET(RWSIM_RUNTIME runtime)
SET(RWSIM_DEVEL development)


SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIM_ROOT}/build/")

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWSIM_ROOT}/build/RobWorkSimConfig.cmake.in
  "${RWSIM_ROOT}/build/RobWorkSimConfig${CMAKE_BUILD_TYPE}.cmake"
)

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWSIM_ROOT}/build/RobWorkSimSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORKSIM_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSIM_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Store the test results in a RWSimConfig.hpp file.
CONFIGURE_FILE(
  ${RWSIM_ROOT}/src/RWSimConfig.hpp.in
  ${RWSIM_ROOT}/src/RobWorkSimConfig.hpp)

CONFIGURE_FILE(
  ${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp.in
  ${RWSIM_ROOT}/src/rwsimlibs/gui/RWSimGuiConfig.hpp)

# process sub directories
#add_subdirectory(ext)
add_subdirectory(src)
add_subdirectory(test)

CONFIGURE_FILE(
  ${RWSIM_ROOT}/build/RobWorkSimBuildConfig.cmake.in
  "${RWSIM_ROOT}/build/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)

CONFIGURE_FILE("${RWSIM_ROOT}/CTestConfig.cmake.in" "${RWSIM_ROOT}/CTestConfig.cmake")
