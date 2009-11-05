# Test CMake version
cmake_minimum_required(VERSION 2.6)

# The name of the project.
project(RobWorkHardware)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
#LINK_LIBRARIES(${ROBWORK_LIBRARIES})

# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )
LINK_DIRECTORIES(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Include directories.
INCLUDE_DIRECTORIES(
  ${ROOT}/ext
  ${ROOT}/src
  ${KATANA_INCLUDE_DIR}
  ${SDH_INCLUDE_DIR}
  ${ESDCAN_INCLUDE_DIR}
)

# TODO: test if boost asio package is available


# Subdirectories to process.
#add_subdirectory(ext)
add_subdirectory(src)
#add_subdirectory(test CMakeTmp/test)
add_subdirectory(example)
