#####################################################
# Example based on simple opengl gui that use some RobWork functionality.
# - the src should contain code for putting into libraries 
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# The name of the project.
PROJECT(rw_viewer)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
ENDIF()

SET(ROBWORK_ROOT ${RW_ROOT})
SET(RobWork_DIR ${ROOT}/../../cmake) 
FIND_PACKAGE(RobWork) 

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROOT}/src)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS}  )

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

FIND_PACKAGE(GLUT)
IF( NOT GLUT_FOUND )
    MESSAGE(FATAL_ERROR "GLUT was not found, which is required by this project!") 
ENDIF ()

INCLUDE_DIRECTORIES(${ROOT}/src/)

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
