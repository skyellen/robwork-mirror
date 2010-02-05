cmake_minimum_required(VERSION 2.6.0)

PROJECT(RWSim)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

#Include default settings for constructing a robwork dependent project
INCLUDE(${ROOT}/build/ProjectSetup.cmake)

#Include default settings for constructing a robwork dependent project
INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# also include stuff from grasping library
INCLUDE_DIRECTORIES( ${GRASPING_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${GRASPING_LIBRARY_DIRS})

# if we have any exe files or plugins that depend on the libraries
INCLUDE_DIRECTORIES( src )

# set up include directories
include_directories(
	./src
    ${BulletIncludes}
    ${OdeIncludes}
)

# test if Bullet exists
if(DEFINED BulletLibs)
	set(RWSIM_HAVE_BULLET 1)
endif()

# test if ODE exists
if(DEFINED OdeLibs)
	set(RWSIM_HAVE_ODE 1)
endif()

# Store the test results in a RWSimConfig.hpp file.
configure_file(
  ${ROOT}/src/RWSimConfig.hpp.in
  ${ROOT}/src/RWSimConfig.hpp)

configure_file(
  ${ROOT}/plugin/RWSimGuiConfig.hpp.in
  ${ROOT}/plugin/RWSimGuiConfig.hpp)


# process sub directories
add_subdirectory(src)
add_subdirectory(plugin)
add_subdirectory(test)
