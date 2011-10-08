# Test CMake version
cmake_minimum_required(VERSION 2.8)

# The name of the project.
PROJECT(RobWorkProject)

SET(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(RW_ROOT ${ROOT}/RobWork/)
SET(RWS_ROOT ${ROOT}/RobWorkStudio/)
SET(RWSIM_ROOT ${ROOT}/RobWorkSim/)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release) 
ENDIF()

add_subdirectory(RobWork)
add_subdirectory(RobWorkStudio)
#add_subdirectory(RobWorkSim)

#add_dependencies(RobWorkStudio RobWork)
#add_dependencies(RobWorkSim RobWork RobWorkStudio)

#add_subdirectory(RobWorkHardware)
#add_subdirectory(RobWorkData)

# Packaging
#INCLUDE(build/packing.cmake)

