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

#set(CMAKE_VERBOSE_MAKEFILE True)

SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
SET(RobWork_DIR "${RW_ROOT}/cmake") 
SET(RobWorkStudio_DIR "${RWSTUDIO_ROOT}/cmake") 

# We use the settings that robwork studio uses
IF( NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE Release)
endif()

FIND_PACKAGE(RobWork REQUIRED)
FIND_PACKAGE(RobWorkStudio REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Include default settings for constructing a robworkstudio dependent project
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

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

#FIND_PACKAGE(Boost COMPONENTS asio) 
#IF( Boost_ASIO_FOUND )
# MESSAGE(STATUS "ASIO found - plugin will not be built!")
#ELSE()
# MESSAGE(SEND_ERROR  "ASIO not found - - plugin will not be built!")
#ENDIF()


# if we want to use ui files add them here
SET(UIS_FILES SamplePlugin.ui )

#Need to add the current binary dir to the include directory because the generated source files are placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT5_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
	QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
	QT4_WRAP_CPP(MocSrcFiles SamplePlugin.hpp)
	QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
ENDIF()

SET(SrcFiles SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} )
#TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ws2_32) # windows
