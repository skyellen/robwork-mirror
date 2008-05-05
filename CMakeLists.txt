# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWork)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# A shorter alias for this directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Extra compiler flags for:
if (CMAKE_COMPILER_IS_GNUCXX)
  # Unix compilers:
  if (DEFINED MINGW)
    set(ExtraCxxFlags "-Wall -fmessage-length=0")
  else ()
    set(ExtraCxxFlags "-Wall -fPIC -fmessage-length=0")
  endif ()
elseif (DEFINED MSVC)
  # Microsoft compilers:
  set(ExtraCxxFlags "")
else ()
  # Other compilers:
  set(ExtraCxxFlags "")
endif ()

# Load the optional RobWork.cmake file.
include(${Root}/RobWork.cmake OPTIONAL)

# Set extra compiler flags
set(CMAKE_CXX_FLAGS "${ExtraCxxFlags} ${CMAKE_CXX_FLAGS}")

# Warning: We no longer use src/RobWork.cmake, but the new RobWork.cmake
# of this directory instead.
if (NOT EXISTS ${Root}/RobWork.cmake)
  message("Warning: No RobWork.cmake file loaded.")
endif ()

# Output goes to bin/ and libs/ unless specified otherwise by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/bin CACHE PATH "Bin directory")
set(LIBRARY_OUTPUT_PATH ${Root}/libs CACHE PATH "Lib directory")

# ?
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
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
