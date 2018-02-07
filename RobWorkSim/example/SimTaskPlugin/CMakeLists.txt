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

# Now set the RobWorkSim directories (edit this if necessary)
SET(RWSIM_ROOT "${ROOT}/../../../RobWorkSim")
SET(RobWorkSim_DIR "${RWSIM_ROOT}/cmake")

#Include RobWorkSim settings

MESSAGE(" RobWorkSim directory: ${RWSIM_ROOT} --")
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSIM_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSIM_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} )


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
SET(UIS_FILES SimTaskPlugin.ui )

# to be able to include the generated ui header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}) 

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT5_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT4_WRAP_CPP(MocSrcFiles SimTaskPlugin.hpp OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

SET(SrcFiles SimTaskPlugin.cpp ${UIS_OUT_H})
SET_SOURCE_FILES_PROPERTIES(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# The shared library to build:
ADD_LIBRARY(SimTaskPlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTaskPlugin ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

