#####################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source 
# files and when configuring your project.  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project. (EDIT THIS)
PROJECT(ConsoleApp)

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
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# setu the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# And now we add any targets that we want
add_executable(SampleTest SampleTest.cpp)
target_link_libraries(SampleTest ${ROBWORK_LIBRARIES})

