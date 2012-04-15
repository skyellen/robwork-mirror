# Test CMake version
cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW)
CMAKE_POLICY(SET CMP0012 NEW) 


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

include(ExternalProject)

########################################################33
# Add RobWork
#
ExternalProject_Add(
   RobWork
   SOURCE_DIR ${RW_ROOT}
   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
   INSTALL_COMMAND "")
# Force configure and build 
ExternalProject_Add_Step(RobWork forcebuild	
    COMMAND ${CMAKE_COMMAND} -E echo_append ""
    COMMENT "Forcing build step for '${proj}'"	
    DEPENDEES configure	
    DEPENDERS build
    ALWAYS 1
)

########################################################33
# include RobWorkStudio
ExternalProject_Add(
   RobWorkStudio
   DEPENDS RobWork
   SOURCE_DIR ${RWS_ROOT}
   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} 
              -DRW_ROOT=${RW_ROOT}
   INSTALL_COMMAND "" 
)
# Force configure and build 
ExternalProject_Add_Step(RobWorkStudio forcebuild	
    COMMAND ${CMAKE_COMMAND} -E echo_append ""
    COMMENT "Forcing build step for '${proj}'"	
    DEPENDEES configure	
    DEPENDERS build
    ALWAYS 1
)


########################################################33
# include RobWorkSim


ExternalProject_Add(
   RobWorkSim
   DEPENDS RobWork RobWorkStudio
   SOURCE_DIR ${RWSIM_ROOT}
   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
              -DRW_ROOT=${RW_ROOT}
              -DRWS_ROOT=${RWS_ROOT}
   INSTALL_COMMAND ""
)

# Force configure and build 
ExternalProject_Add_Step(RobWorkSim forcebuild	
    COMMAND ${CMAKE_COMMAND} -E echo_append ""
    COMMENT "Forcing build step for '${proj}'"	
    DEPENDEES configure	
    DEPENDERS build
    ALWAYS 1
)

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
#add_subdirectory(RobWork)
#add_subdirectory(RobWorkStudio)
#add_subdirectory(RobWorkSim)

#IF( ENABLE_RWHW )
#    add_subdirectory(RobWorkHardware)
#ENDIF() 

#SET(CMAKE_MODULE_PATH ${RWS_ROOT}/cmake ${CMAKE_MODULE_PATH})
#SET(RobWork_DIR ${RW_ROOT}/cmake)
#FIND_PACKAGE(RobWork)
#FIND_PACKAGE(RobWorkStudio)

# we need to explicitly add dependencies
#ADD_DEPENDENCIES(rws ${ROBWORK_LIBRARIES} )
#ADD_DEPENDENCIES(rwsim ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

CONFIGURE_FILE(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile) 

FIND_PROGRAM(doc_executable doxygen)
IF(doc_executable)
    ADD_CUSTOM_TARGET(rwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
ENDIF()


#add_subdirectory(RobWorkHardware)
#add_subdirectory(RobWorkData)

# Packaging
#INCLUDE(build/packing.cmake)

