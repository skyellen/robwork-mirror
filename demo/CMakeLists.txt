# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.4)

# The name of the project.
project(RobWorkDemo)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Load the optional RobWork.cmake file.
include(RobWorkDemo.cmake OPTIONAL)

# It is safer to use absolute directory names.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Output goes to bin/ and lib/ regardless of what is specified by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory" FORCE)

# Include directories.
# (There are no include directories to consider.)

# Subdirectories to process.
# Each subdirectory is an executable. Process them.
add_subdirectory(jtag)
