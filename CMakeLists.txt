# Test CMake version
cmake_minimum_required(VERSION 2.8.12)

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
MESSAGE(STATUS "RobWork ENABLED!")
ExternalProject_Add(
   RobWork
   DOWNLOAD_COMMAND "" 
   SOURCE_DIR ${RW_ROOT}
   CMAKE_ARGS 
   	-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
   	-DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}  
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
OPTION(WITH_RWS "Set true to enable compilation of RobWorkStudio!" TRUE )
IF( WITH_RWS )
    SET(RWSTUDIO_TARGET RobWorkStudio)
    MESSAGE(STATUS "RobWorkStudio ENABLED!")
    ExternalProject_Add(
       RobWorkStudio
       DEPENDS RobWork
       DOWNLOAD_COMMAND "" 
       SOURCE_DIR ${RWS_ROOT}
       CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} 
                  -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
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
ELSE()
    MESSAGE(STATUS "RobWorkStudio DISABLED!")    
ENDIF()

########################################################33
# include RobWorkSim
OPTION(WITH_RWSIM "Set true to enable compilation of RobWorkSim!" TRUE)
IF( WITH_RWSIM )
    MESSAGE(STATUS "RobWorkSim ENABLED!")
    ExternalProject_Add(
       RobWorkSim
       DEPENDS RobWork ${RWSTUDIO_TARGET}
       DOWNLOAD_COMMAND "" 
       SOURCE_DIR ${RWSIM_ROOT}
       CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
       			  -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
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
ELSE()
    MESSAGE(STATUS "RobWorkSim DISABLED!")
ENDIF()

########################################################33
# include RobWorkHardware
OPTION(WITH_RWHW "Set true to enable compilation of RobWorkHardware!" FALSE)
IF( WITH_RWHW )
    MESSAGE(STATUS "RobWorkHardware ENABLED!")
    ExternalProject_Add(
       RobWorkHardware
       DEPENDS RobWork ${RWSTUDIO_TARGET}
       DOWNLOAD_COMMAND "" 
       SOURCE_DIR ${RWHW_ROOT}
       CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                  -DRW_ROOT=${RW_ROOT}
                  -DRWS_ROOT=${RWS_ROOT}
       INSTALL_COMMAND ""
    )
    
    # Force configure and build 
    ExternalProject_Add_Step(RobWorkHardware forcebuild	
        COMMAND ${CMAKE_COMMAND} -E echo_append ""
        COMMENT "Forcing build step for '${proj}'"	
        DEPENDEES configure	
        DEPENDERS build
        ALWAYS 1
    )
ELSE()
    MESSAGE(STATUS "RobWorkHardware DISABLED!")
ENDIF()


########################################################33
# now configure documentation
CONFIGURE_FILE(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile) 

FIND_PROGRAM(doc_executable doxygen
    HINTS
        ${DOXYGEN_PATH}
        $ENV{DOXYGEN_PATH}
)
IF(doc_executable)
    ADD_CUSTOM_TARGET(rwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
ENDIF()

# Packaging
#INCLUDE(build/packing.cmake)

