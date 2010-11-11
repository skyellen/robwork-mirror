#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(LibraryApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE True)

# Now set the robwork root (IMPORTANT EDIT THIS)
IF(WIN32)
    SET(RW_ROOT "$ENV{RW_ROOT}")
    SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")
ELSEIF(UNIX)
    SET(RW_ROOT "/usr/local/RobWork")
    SET(RWSTUDIO_ROOT "/usr/local/RobWorkStudio")
ENDIF()

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Include default settings for constructing a robworkstudio dependent project
INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# setu the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# if we want to use ui files add them here
SET(UIS_FILES SamplePlugin.ui )

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
# Rcc the files:

QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

