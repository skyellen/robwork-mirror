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
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(LibraryApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the robwork root (IMPORTANT EDIT THIS)
SET(RW_ROOT "${ROOT}/../../../RobWork/")

# We use the settings that robwork uses
INCLUDE(${RW_ROOT}/config.cmake OPTIONAL)

SET(CMAKE_BUILD_TYPE Release)
#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# setu the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# add the source files
SET(SRC_FILES
    src/SampleObj1.cpp
    src/SampleObj2.cpp
)

# And now we add any targets that we want
ADD_LIBRARY(samplelibrary  ${SRC_FILES})



FIND_PACKAGE(GLUT)

IF(GLUT_FOUND)




    #
    # Source files
    #
    set(SrcFiles
    	SimpleGLViewer.cpp
    	EventListener.cpp
    	Menu.cpp 
    	MenuItem.cpp 
    	ArcBall.cpp  	  
    )
    
    SET(LIBRARIES
      ${RW_DRAWABLE_LIBRARY_LIST}
      ${RW_PATHPLANNERS_LIBRARY_LIST}
      ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
      ${ROBWORK_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
    
    ADD_LIBRARY(rwtools_simpleviewer ${SrcFiles}) 
    
    # To create an application use
    ADD_EXECUTABLE(ViewerExample ViewerExample.cpp)
    TARGET_LINK_LIBRARIES(ViewerExample rwtools_simpleviewer ${LIBRARIES})
    
    INSTALL(TARGETS ViewerExample DESTINATION bin)
ELSE()
    MESSAGE(STATUS "GLUT not found - SimpleViewer is not added!")
ENDIF()