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
PROJECT(LibraryApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the robwork root (IMPORTANT EDIT THIS)
IF(WIN32)
    SET(RW_ROOT "$ENV{RW_ROOT}")
ELSEIF(UNIX)
    SET(RW_ROOT "/usr/local/RobWork")
ENDIF()

# set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)

SET(CMAKE_BUILD_TYPE Release)
#Include default settings for constructing a robwork dependent project
#Include default settings for constructing a robwork dependent project
SET(ROBWORK_ROOT ${RW_ROOT})
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} ${ROOT}/src)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS}  )

# setu the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# if you have additional libraries or include dirs then add them here
IF(UNIX)
    SET(USER_LIBRARIES glut)
    SET(GLUT_LIBRARIES glut)
ELSE()
    SET(USER_LIBRARIES glut32)
    SET(GLUT_LIBRARIES glut32)
ENDIF()
INCLUDE_DIRECTORIES(${ROOT}/src/)
#LINK_DIRECTORIES( )

#FIND_PACKAGE(GLUT REQUIRED)
#IF( NOT GLUT_FOUND )
#    MESSAGE(FATAL_ERROR "GLUT was not found, which is required by this project!") 
#ENDIF ()

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
