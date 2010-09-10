cmake_minimum_required(VERSION 2.6.0)

IF( CMAKE_VERSION VERSION_LESS 2.6.4 ) 

ELSE( CMAKE_VERSION VERSION_LESS 2.6.4)
    CMAKE_POLICY(SET CMP0011 NEW)
ENDIF( CMAKE_VERSION VERSION_LESS 2.6.4)

PROJECT(RobWorkSim)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${ROOT}/build/")

# test if Bullet exists
SET(RWSIM_HAVE_BULLET False)
OPTION(RWSIM_USE_BULLET "Set to ON if Bullet should be use. you may need to set BULLET_ROOT")
IF(RWSIM_USE_BULLET)
    FIND_PACKAGE(Bullet)
    IF(BULLET_FOUND)
    	SET(RWSIM_HAVE_BULLET TRUE)
    	INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} ${BULLET_ROOT}/Demos/)
        # BULLET_LIBRARIES
        MESSAGE(STATUS "RobWorkSim: Bullet enabled and found.")
    ELSE()
        SET(RWSIM_HAVE_BULLET FALSE)
        MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but not found. Please setup BULLET_ROOT." ${RWSIM_USE_ODE})
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: Bullet disabled.")
ENDIF()

# test if ODE exists
SET(RWSIM_HAVE_ODE False)
OPTION(RWSIM_USE_ODE "Set to ON if ODE should be use. you may need to set ODE_ROOT" ${RWSIM_USE_ODE})
IF(RWSIM_USE_ODE)
    FIND_PACKAGE(ODE)
    IF(ODE_FOUND)
    	SET(RWSIM_HAVE_ODE TRUE)
    	INCLUDE_DIRECTORIES( ${ODE_INCLUDE_DIR} )
    	# ODE_LIBRARIES
        MESSAGE(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
    ELSE()
        SET(RWSIM_HAVE_ODE FALSE)
        MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: ODE disabled.")
ENDIF()

SET(RWSIM_HAVE_RWPHYS False)
OPTION(RWSIM_USE_RWPHYS "Set to ON if ODE should be use. you may need to set ODE_ROOT" ${RWSIM_USE_ODE})
IF(RWSIM_USE_RWPHYS)
	SET(RWSIM_HAVE_RWPHYS TRUE)
    MESSAGE(STATUS "RobWorkSim: RWPhysics enabled and found.")
ENDIF()


# Store the test results in a RWSimConfig.hpp file.
CONFIGURE_FILE(
  ${ROOT}/src/RWSimConfig.hpp.in
  ${ROOT}/src/RWSimConfig.hpp)

CONFIGURE_FILE(
  ${ROOT}/src/rwsim/plugin/RWSimGuiConfig.hpp.in
  ${ROOT}/src/rwsim/plugin/RWSimGuiConfig.hpp)


# process sub directories
add_subdirectory(src)
add_subdirectory(test)


