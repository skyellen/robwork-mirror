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
PROJECT(PluginUIApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE false)

# Now set the RW/RWS root (edit this if necessary)

    SET(RW_ROOT "${ROOT}/../../../RobWork")
    SET(RWS_ROOT "${ROOT}/../../../RobWorkStudio")
    SET(RWSIM_ROOT "${ROOT}/../../../RobWorkSim")

# We use the settings that robwork studio uses
IF( NOT DEFINED CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
#Include default settings for constructing a robwork dependent project
#INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
#INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
#LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Include default settings for constructing a robworkstudio dependent project
#INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
#INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
#LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

#Include RobWorkSim settings

MESSAGE(" ${RWSIM_ROOT} --")
SET(CMAKE_MODULE_PATH ${RWSIM_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSIM_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSIM_LIBRARY_DIRS} )


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

# if we want to use ui files add them here
SET(UIS_FILES SimTaskVisPlugin.ui )

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles SimTaskVisPlugin.hpp)
# Rcc the files:

QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles SimTaskVisPlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SimTaskVisPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskVisPlugin ${ROBWORKSIM_LIBRARIES})

