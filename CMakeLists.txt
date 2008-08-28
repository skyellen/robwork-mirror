# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWork)

# Find and setup OpenGL.
include(FindOpenGL)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Enable new linker path policy.
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif ()

# A shorter alias for this directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Specific compiler settings
include(${Root}/build/RobWork.compiler)

# Load the optional RobWork.cmake file.
include(${Root}/RobWork.cmake OPTIONAL)
if (NOT EXISTS ${Root}/RobWork.cmake)
  # Setup the default settings in case no RobWork.cmake exist. Check out
  # RobWork.cmake.template for description
  include(${Root}/RobWork.cmake.template)
  message(
    "Warning: No RobWork.cmake file loaded, using default settings. "
    "See RobWork.cmake.template.")
endif ()

# Set extra compiler flags
set(CMAKE_CXX_FLAGS "${RW_CXX_FLAGS} ${ExtraCxxFlags} ${CMAKE_CXX_FLAGS}")

# Output goes to bin/ and libs/ unless specified otherwise by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${Root}/libs CACHE PATH "Lib directory" FORCE)

# ?
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
set(CMAKE_LIBRARY_PATH CMAKE_LIBRARY_PATH ${LIBRARY_OUTPUT_PATH})

# Include directories
set(RW_ROOT ${Root})
include(${Root}/build/RobWork.include)

# Subdirectories to process.
add_subdirectory(ext CMakeTmp/ext)
add_subdirectory(src CMakeTmp/src)
add_subdirectory(test CMakeTmp/test)

add_subdirectory(example)
