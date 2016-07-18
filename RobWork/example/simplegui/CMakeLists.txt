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
PROJECT(rw_viewer)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

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

# FreeGlut cannot be found if the following is not set
SET(GLUT_Xi_LIBRARY "")
SET(GLUT_Xmu_LIBRARY "")

FIND_PACKAGE(GLUT REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/)

# add the source files
SET(SRC_FILES
	src/SimpleGLViewer.cpp	
	src/EventListener.cpp
	src/Menu.cpp 
	src/MenuItem.cpp 
	src/ArcBall.cpp  	  
)

ADD_LIBRARY(rwtools_simpleviewer ${SRC_FILES}) 

# set the libraries we need 
SET(LIBRARIES rwtools_simpleviewer ${ROBWORK_LIBRARIES} ${GLUT_LIBRARIES} )

# To create an application use
ADD_EXECUTABLE(ViewerExample ViewerExample.cpp)
TARGET_LINK_LIBRARIES(ViewerExample ${LIBRARIES})

INSTALL(TARGETS ViewerExample DESTINATION bin)
