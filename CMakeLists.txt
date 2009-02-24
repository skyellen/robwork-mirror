# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWork)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Enable new linker path policy.
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif ()

# A shorter alias for this directory.
set(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Specific compiler settings. This initializes RW_CXX_FLAGS, that can be
# modified in the RobWork.cmake template file.
include(${RW_ROOT}/build/compiler.cmake)

# Load the optional RobWork.cmake file.
include(${RW_ROOT}/RobWork.cmake OPTIONAL)
if (NOT EXISTS ${RW_ROOT}/RobWork.cmake)
  # Setup the default settings in case no RobWork.cmake exist.
  include(${RW_ROOT}/RobWork.cmake.template)
  message(
    "Warning: No RobWork.cmake file loaded, using default settings. "
    "See RobWork.cmake.template.")
endif ()


# Set extra compiler flags
set(CMAKE_CXX_FLAGS "${ExtraCxxFlags} ${CMAKE_CXX_FLAGS}")

SET(RW_ROOT ${Root})
 
INCLUDE(${Root}/build/include.cmake)
INCLUDE(${Root}/build/tests.cmake)
INCLUDE(${Root}/build/link.cmake)

# Setup the LibraryList 
SET(LibraryList
  ${SANDBOX_LIB}
  rw_algorithms
  rw_pathplanners
  rw_pathoptimization
  rw_proximitystrategies
  rw_drawable
  rw_simulation
  rw
  ${UblasLibraries}
  ${CollisionDetectionLibraries}
  ${OPENGL_LIBRARIES}
  ${LibraryList}
)

# Output goes to bin/ and libs/ unless specified otherwise by the user.
set(EXECUTABLE_OUTPUT_PATH ${RW_ROOT}/bin CACHE PATH "Bin directory")
set(LIBRARY_OUTPUT_PATH ${RW_ROOT}/libs CACHE PATH "Lib directory")

# Set extra compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RW_CXX_FLAGS}")

# Include directories.
include(${RW_ROOT}/build/include.cmake)

# Tests for the presence of various external tools. This should be run before
# the setup of linker flags below.
include(${RW_ROOT}/build/tests.cmake)

# Link directories and library dependencies.
include(${RW_ROOT}/build/link.cmake)

# Store the test results in a RobWorkConfig.hpp file.
configure_file(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Subdirectories to process:

add_subdirectory(ext CMakeTmp/ext)
add_subdirectory(src CMakeTmp/src)
#add_subdirectory(test CMakeTmp/test)
#add_subdirectory(example)
