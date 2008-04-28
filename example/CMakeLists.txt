# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.4)

# The name of the project.
project(RobWorkExample)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Load the optional RobWork.cmake file.
include(RobWorkExample.cmake OPTIONAL)

# It is safer to use absolute directory names.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Include files for Orocos. Orocos is used by both devices and components.
set(Ext ${PROJECT_SOURCE_DIR}/ext)
include_directories(${Ext}/orocos/include)

# Search for orocos library if it exists then define OROCOS_LIB to true
find_library(OROCOS_LIB orocos-rtt)
if (OROCOS_LIB)
  message("-- Looking for liborocos - found")  
else ()
  message("-- Looking for liborocos - not found")
  message("---- Some examples will not work without orocos!")
endif ()

if ("${EXECUTABLE_OUTPUT_PATH}" STREQUAL "")
  set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory" FORCE)
endif ()
if ("${LIBRARY_OUTPUT_PATH}" STREQUAL "")
  set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory" FORCE)
endif ()

# Add orocos dependant sub directories
if (OROCOS_LIB)
  add_definitions(-DOROCOS_TARGET=gnulinux)
  add_subdirectory(components)
endif ()

# Include directories.
# (There are no include directories to consider.)

# Subdirectories to process.
add_subdirectory(tul)
