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
    #set(ExtraCxxFlags "-Wall -fPIC -fmessage-length=0")
    set(ExtraCxxFlags "-Wall -fmessage-length=0")
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
if (NOT EXISTS ${Root}/RobWork.cmake)
  # Setup the default settings in case no RobWork.cmake exist. Check out
  # RobWork.cmake.template for description
  include(${Root}/RobWork.cmake.template)
  message("Warning: No RobWork.cmake file loaded, using default settings. See RobWork.cmake.template.")
endif ()

# Set extra compiler flags
set(CMAKE_CXX_FLAGS "${ExtraCxxFlags} ${CMAKE_CXX_FLAGS}")


# Output goes to bin/ and libs/ unless specified otherwise by the user.
set(EXECUTABLE_OUTPUT_PATH ${Root}/bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${Root}/libs CACHE PATH "Lib directory" FORCE)

# ?
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
set(CMAKE_LIBRARY_PATH CMAKE_LIBRARY_PATH ${LIBRARY_OUTPUT_PATH})

# Visual C++ definitions
if (DEFINED MSVC)
  # This should be harmless, and is needed to compile things.
  add_definitions(-DNOMINMAX)

  # Without this Boost define for Visual Studio we can't link with lapack.
  add_definitions(-DBIND_FORTRAN_LOWERCASE_UNDERSCORE)
endif ()

# Include directories.
include_directories(
  ${Root}/ext
  ${Root}/ext/boostbindings
  ${Root}/src
  ${Root}/ext/lua/src
  ${Root}/ext/tolua/include)

# Subdirectories to process.
add_subdirectory(ext CMakeTmp/ext)
add_subdirectory(src CMakeTmp/src)
add_subdirectory(test CMakeTmp/test)

add_subdirectory(example)
