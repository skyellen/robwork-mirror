# Test CMake version
cmake_minimum_required(VERSION 2.8)

# The name of the project.
PROJECT(RobWorkProject)

SET(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(RW_ROOT ${ROOT}/RobWork/)
SET(RWS_ROOT ${ROOT}/RobWorkStudio/)
SET(RWSIM_ROOT ${ROOT}/RobWorkSim/)

SET(RWHW_ROOT ${ROOT}/RobWorkHardware/)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release) 
ENDIF()






#include(ExternalProject)
#ExternalProject_Add(
#   RobWork
#   SOURCE_DIR ${RW_ROOT}
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)

#ExternalProject_Add(
#   RobWorkStudio
#   SOURCE_DIR ${RWS_ROOT}
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)

#ExternalProject_Add(
#   RobWorkSim
#   SOURCE_DIR ${RWSIM_ROOT}
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)

#ExternalProject_Add(
#   RobWorkHardware
#   SOURCE_DIR ${RWHW_ROOT}
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)

#add_dependencies(RobWorkStudio RobWork)
#add_dependencies(RobWorkSim RobWork RobWorkStudio)
#add_dependencies(RobWorkHardware RobWork RobWorkStudio)


#MESSAGE(${ROBWORK_VERSION})
add_subdirectory(RobWork)
add_subdirectory(RobWorkStudio)
add_subdirectory(RobWorkSim)

SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${RWS_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
FIND_PACKAGE(RobWorkStudio)

# we need to explicitly add dependencies
ADD_DEPENDENCIES(rwstudio ${ROBWORK_LIBRARIES} )
ADD_DEPENDENCIES(rwsim ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

CONFIGURE_FILE(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile) 

FIND_PROGRAM(doc_executable doxygen)
IF(doc_executable)
    ADD_CUSTOM_TARGET(rwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/Documentation)
ENDIF()


#add_subdirectory(RobWorkHardware)
#add_subdirectory(RobWorkData)

# Packaging
#INCLUDE(build/packing.cmake)

