# Test CMake version
cmake_minimum_required(VERSION 2.6)

# The name of the project.
project(RobWorkStudio)
SET(ROBWORKSTUDIO_VERSION 0.4.0)
STRING( REGEX MATCHALL "[0-9]+" ROBWORKSTUDIO_VERSIONS ${ROBWORKSTUDIO_VERSION} )
LIST( GET ROBWORKSTUDIO_VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

# We want to install in same directory as RobWork
STRING( REGEX REPLACE "[/][^/]*([/]$|$)" "/RobWork" output ${CMAKE_INSTALL_PREFIX})
SET(CMAKE_INSTALL_PREFIX ${output})

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(RWSTUDIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it 
SET(RWS_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
SET(RWS_LIB_INSTALL_DIR "libs/${CMAKE_BUILD_TYPE}/")
SET(RWS_EXT_INSTALL_DIR "ext/")
SET(RWS_HPP_INSTALL_DIR "src/")
SET(RWS_RUNTIME runtime)
SET(RWS_DEVEL development)

#Include default settings for constructing a robwork dependent project
SET(ROBWORK_ROOT ${RW_ROOT})
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

FIND_PACKAGE(Boost COMPONENTS thread program_options  REQUIRED)
IF(Boost_FOUND)    
    LIST(APPEND ROBWORK_LIBRARIES ${Boost_LIBRARIES})
ENDIF ()

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# since rwstudio depends on RobWork we start by setting up rwstudio
INCLUDE(${RWSTUDIO_ROOT}/build/RobWorkStudioSetup.cmake)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

#ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

SET(RW_STATIC_LINK_PLUGINS ${RWS_USE_STATIC_LINK_PLUGINS})
# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RWSTUDIO_ROOT}/src/RobWorkStudioConfig.hpp.in
  ${RWSTUDIO_ROOT}/src/RobWorkStudioConfig.hpp)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWSTUDIO_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWSTUDIO_ROOT}/build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# Documentation
#FIND_PACKAGE(Doxygen)
#IF( DOXYGEN_FOUND )
#    ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${ROOT}/Doxyfile)
#ELSE ()
#    MESSAGE(STATUS "Documentation target is not added! To enable doxygen, make sure doxygen.exe is in the path.")
#ENDIF( )

# Packaging
INCLUDE(build/packing.cmake)


