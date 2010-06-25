# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWorkHardware)

SET(ROBWORKHARDWARE_VERSION 0.4.0)
STRING( REGEX MATCHALL "[0-9]+" ROBWORKHARDWARE_VERSIONS ${ROBWORKHARDWARE_VERSION} )
LIST( GET ROBWORKHARDWARE_VERSIONS 0 ROBWORKHARDWARE_VERSION_MAJOR)
LIST( GET ROBWORKHARDWARE_VERSIONS 1 ROBWORKHARDWARE_VERSION_MINOR)
LIST( GET ROBWORKHARDWARE_VERSIONS 2 ROBWORKHARDWARE_VERSION_PATCH)

# We want to install in same directory as RobWork
STRING( REGEX REPLACE "[/][^/]*([/]$|$)" "/RobWork" output ${CMAKE_INSTALL_PREFIX})
SET(CMAKE_INSTALL_PREFIX ${output})

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(ROBWORKHARDWARE_ROOT ${ROOT})
#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it 
SET(RWHW_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
SET(RWHW_LIB_INSTALL_DIR "libs/${CMAKE_BUILD_TYPE}/")
SET(RWHW_EXT_INSTALL_DIR "ext/")
SET(RWHW_HPP_INSTALL_DIR "src/")
SET(RWHW_RUNTIME runtime)
SET(RWHW_DEVEL development)

#Include default settings for constructing a robwork dependent project
LIST(APPEND CMAKE_MODULE_PATH ${ROOT}/build ${RW_ROOT}/build)
FIND_PACKAGE(RobWork)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )
LINK_DIRECTORIES(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Include directories.
INCLUDE_DIRECTORIES(
  ${ROOT}/ext
  ${ROOT}/src
)

# TODO: test if boost asio package is available

# Enable the RW_ASSERT() macro.
add_definitions(-DRW_ENABLE_ASSERT -fPIC)

# Subdirectories to process.
#add_subdirectory(ext)
add_subdirectory(src)
#add_subdirectory(test CMakeTmp/test)
add_subdirectory(example)
