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
PROJECT(NovintFalcon)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT "/home/nadezda/workspace/rwtrunk/RobWork")

SET(RWSTUDIO_ROOT "/home/nadezda/workspace/rwtrunk/RobWorkStudio")
SET(RWSIM_ROOT "/home/nadezda/workspace/rwtrunk/RobWorkSim")
SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWorkSim_DIR ${RWSIM_ROOT}/cmake)
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)

# Now set the RW/RWS root (edit this if necessary)
#SET(RobWorkStudio_DIR "/home/nadezda/workspace/rwtrunk/RobWorkStudio")
#SET(RobWorkSim_DIR "/home/nadezda/workspace/rwtrunk/RobWorkSim")

#Include RobWorkSim settings
FIND_PACKAGE(RobWorkStudio REQUIRED)
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS} )

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

set(CMAKE_CXX_FLAGS ${CMAKE_CSS_FLAGS} "-fPIC") 

# if we want to use ui files add them here
#SET(UIS_FILES src/FalconControl.ui )

#RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

#QT4_WRAP_CPP(MocSrcFiles src/FalconControl.hpp )
# Rcc the files:

#QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles
	src/FalconInterface.cpp
)

# The shared library to build:
ADD_LIBRARY(NovintFalcon ${SrcFiles})
TARGET_LINK_LIBRARIES(NovintFalcon ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} libnifalcon.so)

add_executable(falcon-test src/falconTest.cpp)
target_link_libraries(falcon-test ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} NovintFalcon libnifalcon.so)



# also, the plugin:
SET(UIS_FILES plugin/FalconPlugin.ui)
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
QT4_ADD_RESOURCES(RccSrcFiles plugin/resources.qrc)
QT4_WRAP_CPP(MocSrcFiles plugin/FalconPlugin.hpp)
add_library(FalconPlugin MODULE plugin/FalconPlugin.cpp plugin/SimulationTrajectory.cpp ${UIS_OUT_H} ${MocSrcFiles} ${RccSrcFiles})
target_link_libraries(FalconPlugin ${ROBWORKSIM_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} NovintFalcon libnifalcon.so)
