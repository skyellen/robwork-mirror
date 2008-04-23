# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWork)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# A shorter alias for this directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Load the optional RobWork.cmake file.
include(${Root}/RobWork.cmake OPTIONAL)

# Warning: We no longer use src/RobWork.cmake, but the new RobWork.cmake
# of this directory instead.
if (NOT EXISTS ${Root}/RobWork.cmake)
  message("Warning: No RobWork.cmake file loaded.")
endif ()

# Output goes to bin/ and libs/ regardless of what is specified by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${Root}/libs CACHE PATH "Lib directory" FORCE)

# Is this needed for anything?
file(MAKE_DIRECTORY libs)
set(CMAKE_LIBRARY_PATH CMAKE_LIBRARY_PATH ${LIBRARY_OUTPUT_PATH})

# Visual C++ definitions
if (DEFINED MSVC)
  add_definitions(-DNOMINMAX)
  add_definitions(-D_HAS_ITERATOR_DEBUGGING=0)
  add_definitions(-D_SECURE_SCL=0)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DBOOST_TEST_NO_LIB)
endif ()

# Include directories.
include_directories(
  ${Root}/ext
  ${Root}/ext/boostbindings
  ${Root}/src
  ${Root}/ext/lua/src
  ${Root}/ext/tolua/include)

# Subdirectories to process.
add_subdirectory(ext)
add_subdirectory(src)
add_subdirectory(test)

add_subdirectory(example)

# Set the nr of lines that one error message can span to 1 if gcc compiler is used:
# => Moved to RobWork.cmake.template
