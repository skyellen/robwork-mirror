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
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

# The name of the project.
PROJECT(FalconControl)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
SET(RobWorkStudio_DIR "$ENV{RW_ROOT}/../../RobWorkStudio/cmake")
SET(RobWorkSim_DIR "$ENV{RW_ROOT}/../../RobWorkSim/cmake")

#Include RobWorkSim settings
FIND_PACKAGE(RobWorkStudio REQUIRED)
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS} /usr/local/lib)

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
SET(UIS_FILES src/FalconControl.ui )

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles src/FalconControl.hpp )
# Rcc the files:

QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles src/FalconControl.cpp src/FalconInterface.cpp src/IO.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(FalconControl MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(FalconControl ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} libnifalcon.so)

