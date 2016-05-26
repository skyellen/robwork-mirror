# Find and sets up RobWorkStudio
# 
#  ROBWORKSTUDIO_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORKSTUDIO_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORKSTUDIO_LIBRARY_DIRS - List of directories where libraries of RobWork are located. 
#  ROBWORKSTUDIO_FOUND       - True if RobWork was found. (not impl yet)
#
#  RWSTUDIO_ROOT             - If set this defines the root of ROBWORKSTUDIO if not set then it
#                              if possible be autodetected.

# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Get the compiler architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(AMD64 1)
ELSE()
	SET(AMD64 0)
ENDIF()

IF(DEFINED RWSIM_ROOT)
	FILE(TO_CMAKE_PATH ${RWSIM_ROOT} RWSIM_ROOT)
ENDIF()

# Check if RW_ROOT path are setup correctly
FIND_FILE(ROBWORKSIM_FOUND RobWorkSimSetup.cmake 
    ${RWSIM_ROOT}/cmake 
	"../build/"
	"../RobWorkSim/build/"
	"../../RobWorkSim/build/"
	"c:/program files/RobWork/build/"
	"c:/programmer/RobWork/build/"
)

IF(NOT ROBWORKSIM_FOUND)
 MESSAGE(SEND_ERROR "Path to RobWorkSim root (RWSIM_ROOT) is incorrectly setup! \nRWSIM_ROOT == ${RWSIM_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWorkSim path: ${RWSIM_ROOT}")

# get the build configuration of the requested built type
IF(EXISTS ${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake)
	INCLUDE(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake)
ELSE()
	INCLUDE(${RWSIM_ROOT}/cmake/RobWorkSimBuildConfig.cmake)
ENDIF()

SET(ROBWORKSIM_VERSION ${RWSIM_BUILD_WITH_VERSION})

MESSAGE(STATUS "RobWorkSim VERSION: ${ROBWORKSIM_VERSION}")

#Include default settings for constructing a robwork dependent project
SET(ROBWORK_ROOT ${RW_ROOT})
SET(CMAKE_MODULE_PATH ${RW_ROOT}/cmake ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork REQUIRED)

STRING(COMPARE EQUAL "${ROBWORKSIM_VERSION}" "${ROBWORK_VERSION}" COMPATIBLE_VERSION)
IF( NOT COMPATIBLE_VERSION )
    MESSAGE(SEND_ERROR "RobWorkSim: Version of RobWork ${ROBWORK_VERSION} is incompatible with version of RobWorkSim ${ROBWORKSIM_VERSION}")
ENDIF()

#And for the qtgui stuff
SET(ROBWORKSTUDIO_ROOT ${RWS_ROOT})
SET(RWSTUDIO_ROOT ${RWS_ROOT})

SET(CMAKE_MODULE_PATH ${RWS_ROOT}/cmake ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWorkStudio REQUIRED)

STRING(COMPARE EQUAL "${ROBWORKSTUDIO_VERSION}" "${ROBWORKSIM_VERSION}" COMPATIBLE_VERSION)
IF( NOT COMPATIBLE_VERSION )
    MESSAGE(SEND_ERROR "RobWorkSim: Version of RobWorkStudio ${ROBWORKSTUDIO_VERSION} is incompatible with version of RobWorkSim ${ROBWORKSIM_VERSION}")
ENDIF()

IF(NOT DEFINED RWSIM_LINKER_FLAGS)	
	SET(RWSIM_LINKER_FLAGS ""
		CACHE STRING "Change this to force using your own linker
					  flags and not those of RobWorkSim"
	)
ENDIF()

# optional compilation of sandbox
IF (RWSIM_BUILD_WITH_SANDBOX)
    MESSAGE(STATUS "RobWorkSim: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rwsim_sandbox")
ELSE ()
    MESSAGE(STATUS "RobWorkSim: Sandbox DISABLED!")    
    SET(RWSIM_HAVE_SANDBOX false)
ENDIF ()

# test if Bullet exists
OPTION(RWSIM_USE_BULLET "Set to ON if Bullet should be use. you may need to set BULLET_ROOT" ${RWSIM_BUILD_WITH_BULLET})
IF(RWSIM_USE_BULLET)
    IF(RWSIM_BUILD_WITH_BULLET)
        IF(NOT BULLET_ROOT) 
            SET(BULLET_ROOT ${RWSIM_BUILD_WITH_BULLET_ROOT})
        ENDIF()
        IF(NOT BULLET_INCLUDE_DIR) 
            SET(BULLET_INCLUDE_DIR ${RWSIM_BUILD_WITH_BULLET_INCLUDE_DIR})
        ENDIF()
        
        FIND_PACKAGE(Bullet)
        IF(BULLET_FOUND)
        	SET(RWSIM_BULLET_LIBRARY rwsim_bullet)
            # BULLET_LIBRARIES
            MESSAGE(STATUS "RobWorkSim: Bullet enabled and found.")
        ELSE()
            SET(RWSIM_HAVE_BULLET FALSE)
            MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but not found. Please setup BULLET_ROOT." ${RWSIM_USE_ODE})
        ENDIF()
   ELSE()
       MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but RobWorkSim was NOT build with Bullet support! Please recompile RobWorkSim")
   ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: Bullet disabled.")
ENDIF()

# test if ODE exists
OPTION(RWSIM_USE_ODE "Set to ON if ODE should be use. you may need to set ODE_ROOT" ${RWSIM_BUILD_WITH_ODE})
IF(RWSIM_USE_ODE)
    IF(RWSIM_BUILD_WITH_ODE)
        SET(ODE_USE_DOUBLE ${RWSIM_BUILD_WITH_ODE_USE_DOUBLE})
        SET(ODE_USE_DEBUG ${RWSIM_BUILD_WITH_ODE_USE_DEBUG})
        IF(NOT ODE_DIR)
            SET(ODE_DIR ${RWSIM_BUILD_WITH_ODE_DIR})
        ENDIF()
        IF(NOT ODE_INCLUDE_DIR)
            SET(ODE_INCLUDE_DIR ${RWSIM_BUILD_WITH_ODE_INCLUDE_DIR})
        ENDIF()
        FIND_PACKAGE(ODE)
        IF(ODE_FOUND)
        	SET(RWSIM_ODE_LIBRARY rwsim_ode)
        	# ODE_LIBRARIES
            MESSAGE(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
        ELSE()
            MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
        ENDIF()
   ELSE()
       MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but RobWorkSim was NOT build with ODE support! Please recompile RobWorkSim")
   ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: ODE disabled.")
ENDIF()

# Setup RobWorkSim include and link directories
SET(ROBWORKSIM_INCLUDE_DIR ${RWSIM_ROOT}/src/)
SET(ROBWORKSIM_LIBRARY_DIRS ${RWSIM_LIBS_DIR})
#
# The include dirs
#
SET(ROBWORKSIM_INCLUDE_DIR
    ${RWSIM_ROOT}/src
    ${ROBWORKSTUDIO_INCLUDE_DIR}
    ${ROBWORK_INCLUDE_DIR}
    ${ODE_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIR}
)
#MESSAGE("ODE INC : ${ODE_INCLUDE_DIR}")
#
# The library dirs
#
SET(ROBWORKSIM_LIBRARY_DIRS
    ${Boost_LIBRARY_DIRS}
    ${RWSIM_ROOT}/libs/
    ${RWSIM_ROOT}/libs/${CMAKE_BUILD_TYPE}/     
    ${RWSIM_ARCHIVE_OUT_DIR}
    ${ROBWORKSTUDIO_LIBRARY_DIRS}    
    ${ROBWORK_LIBRARY_DIRS}
    
)

#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORKSIM_LIBRARIES_TMP
  ${RWSIM_SANDBOX}
  rwsim_gui
  ${RWSIM_BULLET_LIBRARY}
  ${RWSIM_ODE_LIBRARY}
  #${RWSIM_LUA}
  rwsim
  ${BULLET_LIBRARIES}
  ${ODE_LIBRARIES}
 # ${ROBWORKSTUDIO_LIBRARIES}
 # ${ROBWORK_LIBRARIES}
)

SET(ROBWORKSIM_LIBRARIES)
FOREACH(l ${ROBWORKSIM_LIBRARIES_TMP})
  UNSET(tmp CACHE)
  FIND_LIBRARY(tmp ${l} PATHS ${ROBWORKSIM_LIBRARY_DIRS})
  IF(tmp)
    LIST(APPEND ROBWORKSIM_LIBRARIES ${tmp})
  ELSE()
    LIST(APPEND ROBWORKSIM_LIBRARIES ${l})
  ENDIF()
ENDFOREACH(l)

set(ROBWORKSIM_LIBRARIES 
    ${ROBWORKSIM_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${ROBWORK_LIBRARIES}
)
