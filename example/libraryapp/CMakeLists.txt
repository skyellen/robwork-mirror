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
PROJECT(LibraryApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the robwork root (IMPORTANT EDIT THIS)
SET(RW_ROOT "${ROOT}/../../../RobWork/")

# We use the settings that robwork uses
INCLUDE(${RW_ROOT}/config.cmake)

SET(CMAKE_BUILD_TYPE Release)
#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/RobWorkSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
#LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
#LINK_LIBRARIES(${ROBWORK_LIBRARIES})

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
ADD_LIBRARY(samplelibrary ${SRC_FILES})

