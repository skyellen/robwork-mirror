# Test CMake version
cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW) 

# The name of the project.
project(RobWorkStudio)
SET(ROBWORKSTUDIO_VERSION 0.5.0)
STRING( REGEX MATCHALL "[0-9]+" ROBWORKSTUDIO_VERSIONS ${ROBWORKSTUDIO_VERSION} )
LIST( GET ROBWORKSTUDIO_VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

MESSAGE(STATUS "RobWorkStudio version ${ROBWORKSTUDIO_VERSION}")

# We want to install in same directory as RobWork
STRING( REGEX REPLACE "[/][^/]*([/]$|$)" "/RobWork" output ${CMAKE_INSTALL_PREFIX})
SET(CMAKE_INSTALL_PREFIX ${output})

# Used to resolve absolute path names
IF (NOT RWS_ROOT)
    SET(RWS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    SET(RWSTUDIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF ()

#Include default settings for constructing a robwork dependent project
SET(ROOT ${RWS_ROOT})
INCLUDE(${RWS_ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it 
SET(RWS_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
SET(RWS_LIB_INSTALL_DIR "libs/${CMAKE_BUILD_TYPE}/")
SET(RWS_EXT_INSTALL_DIR "ext/")
SET(RWS_HPP_INSTALL_DIR "src/")
SET(RWS_RUNTIME runtime)
SET(RWS_DEVEL development)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioConfig${CMAKE_BUILD_TYPE}.cmake")

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWS_ROOT}/build/RobWorkStudioSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
#MESSAGE("${ROBWORKSTUDIO_LIBRARY_DIRS}")

# linking options
SET(RW_STATIC_LINK_PLUGINS ${RWS_USE_STATIC_LINK_PLUGINS})
# linking options
OPTION(RWS_USE_STATIC_LINK_COMPONENTS "Set to false if dynamic link libraries of components is needed" ON)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RWS_ROOT}/src/RobWorkStudioConfig.hpp.in
  ${RWS_ROOT}/src/RobWorkStudioConfig.hpp)

# sub directories to process
ADD_SUBDIRECTORY(src)

OPTION(RWS_BUILD_TESTS "Set when you want to build the tests" ${RWS_BUILD_TESTS})
IF( RWS_BUILD_TESTS )
    MESSAGE(STATUS "RobWorkStudio tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWorkStudio tests DISABLED!")
ENDIF()


# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# Documentation
#FIND_PACKAGE(Doxygen)
#IF( DOXYGEN_FOUND )
#    ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${ROOT}/Doxyfile)
#ELSE ()
#    MESSAGE(STATUS "Documentation target is not added! To enable doxygen, make sure doxygen.exe is in the path.")
#ENDIF( )

# Try to find the current revision 
FIND_PACKAGE(Subversion QUIET)
IF(Subversion_FOUND)
    #Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE("Current revision is ${RobWork_WC_REVISION}")
    #Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork) # not foundd in linux
    #MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)


# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION)
CONFIGURE_FILE(
   "${RWSTUDIO_ROOT}/build/dependencies.cmake.in"
   "${RWSTUDIO_ROOT}/build/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RWSTUDIO_ROOT}/build/dependencies.cmake")
#include(${RWSTUDIO_ROOT}/build/dependencies.cmake)

# Packaging
INCLUDE(build/packing.cmake)


