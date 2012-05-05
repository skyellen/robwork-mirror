cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW)
CMAKE_POLICY(SET CMP0012 NEW) 

PROJECT(RobWorkSim)
SET(ROBWORKSIM_VERSION 0.5.1)
SET(RWSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
MESSAGE(STATUS "RobWorkSim version ${ROBWORKSIM_VERSION}")

# lets start by finding RobWork since this is the primary dependency and because we
# want to use the macros available in RobWork
# if RW_ROOT is defined then we use that, else we try finding it the traditional way
IF(RW_ROOT)
    SET(RobWork_DIR ${RW_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKDEVEL_FOUND RobWorkSetup.cmake 
        ${RWSIM_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RobWork_DIR ${RWSIM_ROOT}/../RobWork/cmake)    
    ENDIF()
ENDIF()
FIND_PACKAGE(RobWork ${ROBWORK_VERSION} REQUIRED)

# include the macros from robwork
INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)
#INCLUDE(${RWS_ROOT}/cmake/RobWorkStudioMacros.cmake)
# for backward compatibility
SET(ROOT RWSIM_ROOT)
RW_INIT_PROJECT(${RWSIM_ROOT} RobWorkSim RWSIM ${ROBWORKSIM_VERSION})
RW_GET_OS_INFO()
RW_SET_INSTALL_DIRS(RobWorkSim RWSIM)
RW_OPTIONS(RWSIM)

# add the RWSIM modeules to the module path
SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${RWSIM_ROOT}/cmake/Modules/")

# This sets up ROBWORKSIM_INCLUDE_DIR and ROBWORKSIM_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWSIM_ROOT}/cmake/RobWorkSimSetup.cmake)

# now lets find RobWorkStudio
# if RWS_ROOT is defined then we use that, else we try finding it the traditional way
IF(RWS_ROOT)
    SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKSTUDIODEVEL_FOUND RobWorkStudioSetup.cmake 
        ${RWSIM_ROOT}/../RobWorkStudio/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RobWorkStudio_DIR ${RWSIM_ROOT}/../RobWorkStudio/cmake)    
    ENDIF()
ENDIF()
MESSAGE("RWS_ROOT ${RobWorkStudio_DIR}")
FIND_PACKAGE(RobWorkStudio ${ROBWORKSIM_VERSION} REQUIRED)

# Configure build/RobWorkSimConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWSIM_ROOT}/cmake/RobWorkSimConfig.cmake.in
  "${RWSIM_ROOT}/cmake/RobWorkSimConfig${CMAKE_BUILD_TYPE}.cmake"
)


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


#############################################################
# setup testing
#
INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RWSIM_IS_TESTS_ENABLED "Set when you want to build the tests" 
                   ON "${RWSIM_BUILD_TESTS}" OFF)
IF( RWSIM_IS_TESTS_ENABLED )
    MESSAGE(STATUS "RobWork tests ENABLED!")
    # TODO Specify SET(SITE "myname") to configure the site name to use when uploading 
    SET(BUILDNAME "${INFO}")
    ENABLE_TESTING()
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWork tests DISABLED!")
ENDIF()

CONFIGURE_FILE(
  ${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake.in
  "${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)

# configure testing configuration
CONFIGURE_FILE("${RWSIM_ROOT}/CTestConfig.cmake.in" "${RWSIM_ROOT}/CTestConfig.cmake")
CONFIGURE_FILE("${RWSIM_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
