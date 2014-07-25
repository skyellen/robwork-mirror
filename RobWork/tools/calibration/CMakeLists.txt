#####################################################
# Example based on simple opengl gui that use some RobWork functionality.
# - the src should contain code for putting into libraries 
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################


# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. 
PROJECT(rw_calibration_tools)

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )


SET(CALIBRATION_DEPENDENCIES rw_calibration)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/)

ADD_EXECUTABLE( rw_calibration-tool2 CalibrationTool2.cpp )
TARGET_LINK_LIBRARIES( rw_calibration-tool2 ${ROBWORK_LIBRARIES} )  
ADD_DEPENDENCIES( rw_calibration-tool2 ${CALIBRATION_DEPENDENCIES} )

ADD_EXECUTABLE( rw_calibration-mtool MeasurementTool.cpp )
TARGET_LINK_LIBRARIES( rw_calibration-mtool ${ROBWORK_LIBRARIES} )
ADD_DEPENDENCIES( rw_calibration-mtool ${CALIBRATION_DEPENDENCIES} )

ADD_EXECUTABLE( rw_calibration-verification-tool CalibrationVerificationTool.cpp )
TARGET_LINK_LIBRARIES( rw_calibration-verification-tool ${ROBWORK_LIBRARIES} )
ADD_DEPENDENCIES( rw_calibration-verification-tool ${CALIBRATION_DEPENDENCIES} )


