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

set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory")
set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory")

# Include directories.
# (There are no include directories to consider.)

# Subdirectories to process.
add_subdirectory(DynamicLoading)
add_subdirectory(tul)
