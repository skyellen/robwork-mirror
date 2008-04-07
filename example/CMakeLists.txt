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

# Add orocos dependant sub directories
if (OROCOS_LIB)
  add_subdirectory(components)
endif ()

# Output goes to bin/ and lib/ regardless of what is specified by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory" FORCE)

# Include directories.
# (There are no include directories to consider.)

# Subdirectories to process.
add_subdirectory(tul)