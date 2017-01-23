###########################################
# Test CMake version and CMAKE_BUILD_TYPE
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF()

###########################################
# The name of the project.
project(RobWorkHardware)
SET(ROBWORKHARDWARE_VERSION 0.7.0)
SET(RWHW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
MESSAGE(STATUS "RobWorkHardware version ${ROBWORKHARDWARE_VERSION}")

###########################################
# lets start by finding RobWork since this is the primary dependency and because we
# want to use the macros available in RobWork
# if RW_ROOT is defined then we use that, else we try finding it the traditional way
IF(RW_ROOT)
    SET(RobWork_DIR ${RW_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKDEVEL_FOUND RobWorkSetup.cmake 
        ${RWHW_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RW_ROOT ${RWHW_ROOT}/../RobWork/)
        SET(RobWork_DIR ${RW_ROOT}/cmake)
    ENDIF()
ENDIF()
FIND_PACKAGE(RobWork ${ROBWORKHARDWARE_VERSION} REQUIRED)

###########################################
# include the macros from robwork
INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)
#INCLUDE(${RWS_ROOT}/cmake/RobWorkStudioMacros.cmake)

RW_INIT_PROJECT(${RWHW_ROOT} RobWorkHardware RWHW ${ROBWORKHARDWARE_VERSION})
RW_GET_OS_INFO()
RW_SET_INSTALL_DIRS(RobWork RWHW)
RW_OPTIONS(RWHW)

###########################################
# Add an "uninstall" target
CONFIGURE_FILE("${RWHW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall "${CMAKE_COMMAND}" -P
                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

###########################################
# setup stuff for RobWorkHardware

# Load the optional Default.cmake file.
#INCLUDE(${ROOT}/config.cmake OPTIONAL)
#IF (NOT EXISTS ${RWHW_ROOT}/config.cmake)
#  # Setup the default settings in case no RobWork.cmake exist.
#  INCLUDE(${RWHW_ROOT}/config.cmake.template)
#  MESSAGE(STATUS " No config.cmake file loaded, using default settings from config.cmake.template")
#ENDIF ()

# make sure that libraries can reach the cmake modules
SET(CMAKE_MODULE_PATH ${RWHW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

# now we collect all added libraries in an environment variable
set( ENV{ROBWORKHARDWARE_LIBRARIES} "")

# also make sure flags from robwork is also used in robworkhardware
SET(RWHW_CXX_FLAGS "${RW_BUILD_WITH_CXX_FLAGS}" 
	CACHE STRING "Change this to force using your own flags and not those of RobWorkHardware"
)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWHW_CXX_FLAGS}")

# setup general include and link stuff
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )
LINK_DIRECTORIES(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Include directories.
INCLUDE_DIRECTORIES(
  ${ROOT}/ext
  ${ROOT}/src
)

# setup the install dirs TODO should be cached to allow the use to change it 
#SET(RWHW_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
#SET(RWHW_LIB_INSTALL_DIR "libs/${CMAKE_BUILD_TYPE}/")
#SET(RWHW_EXT_INSTALL_DIR "ext/")
#SET(RWHW_HPP_INSTALL_DIR "src/")
#SET(RWHW_RUNTIME runtime)
#SET(RWHW_DEVEL development)

###########################################
# now add all source directories to process.
add_subdirectory(ext)
add_subdirectory(src)
#add_subdirectory(test CMakeTmp/test)
add_subdirectory(example)

###########################################
# testing 
RW_SYS_INFO(INFO)
MESSAGE(STATUS "RobWorkStudio: platform ${INFO} ")

CONFIGURE_FILE("${RWHW_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RWHW_IS_TESTS_ENABLED "Set when you want to build the tests" 
                   ON "${RWHW_BUILD_TESTS}" OFF)
IF( RWS_IS_TESTS_ENABLED )
    MESSAGE(STATUS "RobWorkHardware: tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWorkHardware: tests DISABLED!")
ENDIF()

###############################################################
# Try to find the current revision 
FIND_PACKAGE(Subversion QUIET)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${RWHW_ROOT} RobWorkHardware)
    MESSAGE(STATUS "RobWorkHardware: Current revision is ${RobWorkHardware_WC_REVISION}")
    SET(ROBWORKHARDWARE_REVISION ${RobWorkHardware_WC_REVISION})
ELSE()
    MESSAGE(STATUS "RobWorkHardware: No revision info!")
ENDIF(Subversion_FOUND)

###############################################################
# CONFIGURATION
# configure build/RobWorkHardwareConfig.cmake.in 


# first configure the header file
CONFIGURE_FILE(
  ${RWHW_ROOT}/src/RobWorkStudioConfig.hpp.in
  "${RWHW_ROOT}/src/rwhw/RobWorkHardwareConfig.hpp"
  @ONLY
)

# next build information script
# make a list of all enabled components 
SET( RWHW_ENABLED_COMPONENTS "")
foreach(_name ${RW_SUBSYSTEMS})
IF( ${BUILD_${_name}} )
    LIST(APPEND RWHW_ENABLED_COMPONENTS ${_name})
ENDIF()
endforeach() 
# TODO these depend on packages...
SET(ROBWORKHARDWARE_LIBRARY_DIRS ${RWHW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
SET(ROBWORKHARDWARE_INCLUDE_DIR ${RWHW_ROOT}/src)
SET(ROBWORKHARDWARE_LIBRARIES $ENV{ROBWORKHARDWARE_LIBRARIES})
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareBuildConfig_${RW_BUILD_TYPE}.cmake"
  @ONLY
)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robworkstudio
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareConfig.cmake" @ONLY)

# and the version info
CONFIGURE_FILE(
  ${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake.in
  "${RWHW_ROOT}/cmake/RobWorkHardwareConfigVersion.cmake"
)

