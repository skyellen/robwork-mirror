# Test CMake version
cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(SDH)

# Allow the syntax else (), endif (), etc.
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# A shorter alias for this directory.
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

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

# Set extra compiler flags
set(CMAKE_CXX_FLAGS "${ExtraCxxFlags} ${CMAKE_CXX_FLAGS}")

if (DEFINED CYGWIN)
    MESSAGE(STATUS "CYGWIN")
elseif(DEFINED MSVC)
    MESSAGE(STATUS "MSVC")
    add_definitions(-DSDH_USE_VCC)
    SET(SDH_USE_VCC on)
elseif(DEFINED WIN32)
    # this is for MINGW
    MESSAGE(STATUS "MINGW")
    SET(SDH_USE_VCC on)
    SET(WITH_ESD_CAN on)
    add_definitions(-DSDH_USE_VCC)
    add_definitions(-DWITH_ESD_CAN)
else ()
    MESSAGE(STATUS "NONE")
     SET(WITH_ESD_CAN on)
    add_definitions(-DWITH_ESD_CAN)
endif () 


# make sure to setup include and lib directories
include_directories(${ROOT}/sdh/ )
include_directories(${ROOT})
link_directories( ${ROOT}/libs/ )

INCLUDE(${ROOT}/../../build/FindESDCAN.cmake)
include_directories(${ESDCAN_INCLUDE_DIR})
LINK_LIBRARIES(${ESDCAN_LIBRARY})
#link_libraries(${ROOT}/../esd/ntcan.lib)
#link_libraries(${ROOT}/../esd/ntcan.lib)
#include_directories("C:/Program Files/ESD/CAN/SDK/include")
#link_directories("C:/Program Files/ESD/CAN/SDK/lib/vc/i386")

# Set default library path
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${ROOT}/libs/) 

# Output goes to bin/ and libs/ unless specified otherwise by the user.
set(EXECUTABLE_OUTPUT_PATH ${ROOT}/bin CACHE PATH "Bin directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${ROOT}/libs CACHE PATH "Lib directory" FORCE)

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

# Store the test results in a SDHConfig.hpp file.
CONFIGURE_FILE(
  ${ROOT}/SDHConfig.hpp.in
  ${ROOT}/sdh/SDHConfig.hpp)


# Include directories.

# Subdirectories to process.
add_subdirectory(sdh CMakeTmp/sdh)
add_subdirectory(demo CMakeTmp/demo)
#if (DEFINED MSVC)
#    add_subdirectory(demo CMakeTmp/demo) 
#endif ()

