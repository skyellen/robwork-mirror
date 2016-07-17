# Find and sets up RobWorkSim. 
# 
#
#
#
#  ROBWORKSIM_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORKSIM_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORKSIM_LIBRARY_DIRS - List of directories where libraries of RobWork are located. 
#  ROBWORKSIM_FOUND       - True if RobWork was found. (not impl yet)
#
#  RWSIM_ROOT             - If set this defines the root of ROBWORKSIM if not set then it
#                              if possible be autodetected.
#
#  
#

# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Check if RWstudio_ROOT path are setup correctly
FIND_FILE(ROBWORKSIM_FOUND RobWorkSimSetup.cmake ${RWSIM_ROOT}/cmake NO_DEFAULT_PATH)
IF(NOT ROBWORKSIM_FOUND)
 MESSAGE(SEND_ERROR "RobWorkSim: Path to RobWorkSim root (RWSIM_ROOT) is incorrectly setup! \nRWSIM_ROOT == ${RWSIM_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWorkSim: ROOT dir: ${RWSIM_ROOT}")

#
# Setup the default include and library dirs for RobWorkSim
#
#INCLUDE("${RWSIM_ROOT}/build/RobWorkSimBuildConfig${CMAKE_BUILD_TYPE}.cmake")


 ####################################################################
# DEPENDENCIES - REQUIRED
# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
#
 
#Include default settings for constructing a robwork dependent project
#SET(ROBWORK_ROOT ${RW_ROOT})
#SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
#SET(RobWork_DIR ${RW_ROOT}/cmake)
#FIND_PACKAGE(RobWork REQUIRED)

#STRING(COMPARE EQUAL "${ROBWORKSIM_VERSION}" "${ROBWORK_VERSION}" COMPATIBLE_VERSION)
#IF( NOT COMPATIBLE_VERSION )
#    MESSAGE(SEND_ERROR "RobWorkSim: Version of RobWork ${ROBWORK_VERSION} is incompatible with version of RobWorkSim ${ROBWORKSIM_VERSION}")
#ENDIF()

#And for the qtgui stuff
#SET(ROBWORKSTUDIO_ROOT ${RWS_ROOT})
#SET(RWSTUDIO_ROOT ${RWS_ROOT})

#SET(CMAKE_MODULE_PATH ${RWS_ROOT}/cmake ${CMAKE_MODULE_PATH})
#FIND_PACKAGE(RobWorkStudio REQUIRED)

#STRING(COMPARE EQUAL "${ROBWORKSTUDIO_VERSION}" "${ROBWORKSIM_VERSION}" COMPATIBLE_VERSION)
#IF( NOT COMPATIBLE_VERSION )
#    MESSAGE(SEND_ERROR "RobWorkSim: Version of RobWorkStudio ${ROBWORKSTUDIO_VERSION} is incompatible with version of RobWorkSim ${ROBWORKSIM_VERSION}")
#ENDIF()


 ####################################################################
# DEPENDENCIES - OPTIONAL
# these dependencies are optional, which is the user can switch off
# modules
  
# optional compilation of sandbox
IF (RWSIM_BUILD_SANDBOX)
    MESSAGE(STATUS "RobWorkSim: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rwsim_sandbox")
    SET(RWSIM_HAVE_SANDBOX true)
ELSE ()
    MESSAGE(STATUS "RobWorkSim: Sandbox DISABLED!")    
    SET(RWSIM_HAVE_SANDBOX false)
ENDIF ()

INCLUDE(CMakeDependentOption)
# optional compilation of sandbox
SET(RWSIM_HAVE_LUA False)
CMAKE_DEPENDENT_OPTION(RWSIM_DISABLE_LUA "Set when you want to disable lua!" OFF "RW_BUILD_WITH_LUA" ON)
IF( NOT RWSIM_DISABLE_LUA )
    IF (RW_BUILD_WITH_LUA)
        MESSAGE(STATUS "RobWorkSim: Lua ENABLED!")
        SET(RWSIM_LUA "rwsim_lua")
        SET(RWSIM_HAVE_LUA True)
    ELSE ()
        MESSAGE(STATUS "RobWorkSim: Lua DISABLED! - RobWork is NOT compiled with Lua support! Set RWSIM_DISABLE_LUA=ON")
        SET(RWSIM_HAVE_LUA False)    
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWorkSim: Lua DISABLED!")
ENDIF()
#MESSAGE(" ${RWSIM_DISABLE_LUA} and ${RW_BUILD_WITH_LUA} ") 

# test if Bullet exists
SET(RWSIM_HAVE_BULLET False)
#OPTION(RWSIM_USE_BULLET "Set to ON if Bullet should be use. you may need to set BULLET_ROOT")
FIND_PACKAGE(Bullet QUIET)
IF(NOT DEFINED RWSIM_DISABLE_BULLET)
    SET(RWSIM_DISABLE_BULLET OFF)
ENDIF()
CMAKE_DEPENDENT_OPTION(RWSIM_DISABLE_BULLET "Set when you want to disable Bullet!" ${RWSIM_DISABLE_BULLET} "BULLET_FOUND" ON)
IF(NOT RWSIM_DISABLE_BULLET)
    FIND_PACKAGE(Bullet)
    IF(BULLET_FOUND)
    	SET(RWSIM_HAVE_BULLET TRUE)
    	#INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} ${BULLET_ROOT}/Demos/)
    	SET(RWSIM_BULLET_LIBRARY rwsim_bullet ${BULLET_LIBRARIES})

        # BULLET_LIBRARIES
        MESSAGE(STATUS "RobWorkSim: Bullet enabled and found.")
        SET(RW_BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR})
    ELSE()
        SET(RWSIM_HAVE_BULLET FALSE)
        MESSAGE(SEND_ERROR "RobWorkSim: Bullet enabled but not found. Please setup BULLET_ROOT." ${RWSIM_USE_BULLET})
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: Bullet disabled.")
ENDIF()

# test if ODE exists
SET(RWSIM_HAVE_ODE False)
FIND_PACKAGE(ODE QUIET)
#MESSAGE("ODE_FOUND: ${ODE_FOUND} ")
CMAKE_DEPENDENT_OPTION(RWSIM_DISABLE_ODE "Set when you want to disable ODE!" OFF "${ODE_FOUND}" ON)
#OPTION(RWSIM_USE_ODE "Set to ON if ODE should be use. you may need to set ODE_ROOT" ${RWSIM_USE_ODE})
IF(NOT RWSIM_DISABLE_ODE)
    FIND_PACKAGE(ODE)
    IF(ODE_FOUND)
    	SET(RWSIM_HAVE_ODE TRUE)
    	#INCLUDE_DIRECTORIES( ${ODE_INCLUDE_DIR} )
    	SET(RWSIM_ODE_LIBRARY rwsim_ode ${ODE_LIBRARIES})
    	# ODE_LIBRARIES
        MESSAGE(STATUS "RobWorkSim: ODE enabled and found. Using ${ODE_BUILD_WITH}")
        SET(RW_ODE_INCLUDE_DIR ${ODE_INCLUDE_DIR} )
    ELSE()
        SET(RWSIM_HAVE_ODE FALSE)
        MESSAGE(SEND_ERROR "RobWorkSim: ODE enabled but not found. Please setup ODE_ROOT.")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkSim: ODE disabled.")
ENDIF()

SET(RWSIM_HAVE_RWPE False)
OPTION(RWSIM_DISABLE_RWPE "Set when you want to disable RobWorkPhysicsEngine!" OFF)
IF(NOT RWSIM_DISABLE_RWPE)
	SET(RWSIM_HAVE_RWPE TRUE)
	SET(RWSIM_RWPE_LIBRARY rwsim_rwpe)
    MESSAGE(STATUS "RobWorkSim: RobWorkPhysicsEngine enabled.")
ENDIF()

SET(RWSIM_HAVE_RWPHYS False)
OPTION(RWSIM_DISABLE_RWPHYS "Set when you want to disable RWPhysics engine!" OFF)
IF(NOT RWSIM_DISABLE_RWPHYS)
	SET(RWSIM_HAVE_RWPHYS TRUE)
    MESSAGE(STATUS "RobWorkSim: RWPhysics enabled.")
ENDIF()

# Add additional packages that are required by your project here
IF( USE_OPENCV AND DEFINED OpenCV_ROOT_DIR)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ROOT}/cmake/Modules)

    SET(OpenCV_FIND_REQUIRED_COMPONENTS CV CXCORE HIGHGUI)
    FIND_PACKAGE( OpenCV REQUIRED "CV CXCORE HIGHGUI")
    INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )
    MESSAGE(STATUS "RobWorkSim: USING OPENCV")
ENDIF()


#######################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Set extra compiler flags. The user should be able to change this. 
# The compiler flags from RobWork are automatically set 
#
IF(NOT DEFINED RWSIM_CXX_FLAGS)
  
	SET(RWSIM_CXX_FLAGS ${RWSIM_CXX_FLAGS_TMP} 
		CACHE STRING "Change this to force using your own flags and not those of RobWorkSim"
	)
ENDIF()
ADD_DEFINITIONS(${RWSIM_CXX_FLAGS})
MESSAGE(STATUS "RobWorkSim: Adding RWSIM CXX flags: ${RWSIM_CXX_FLAGS}")

#
# Set extra linker flags. The user should be able to change this. 
# The linker flags from RobWork are automatically set.
#
IF(DEFINED RWSIM_LINKER_FLAGS)
	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RWSIM_LINKER_FLAGS}" CACHE STRING "" FORCE)
	IF(WIN32)
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RWSIM_LINKER_FLAGS}" CACHE STRING "" FORCE)
	ENDIF()
	
	MESSAGE(STATUS "RobWorkSim: Adding RWSIM linker flags: ${RWSIM_LINKER_FLAGS}")
ENDIF()

###########################################################################
# SETTING UP VARS
# here we setup the output variables
# 

# Setup RobWorkSim include and link directories
#
# The include dirs
#
SET(ROBWORKSIM_INCLUDE_DIRS
    ${RWSIM_ROOT}/src
    ${ROBWORK_INCLUDE_DIRS}
    ${RW_ODE_INCLUDE_DIR}
    ${RW_BULLET_INCLUDE_DIR}
)
SET(ROBWORKSIM_INCLUDE_DIR "${ROBWORKSIM_INCLUDE_DIRS}")
#
# The library dirs
#
SET(ROBWORKSIM_LIBRARY_DIRS
    ${Boost_LIBRARY_DIRS}
    ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    ${RWSIM_LIBRARY_OUT_DIR} 
    ${RWSIM_ARCHIVE_OUT_DIR}
    ${ROBWORK_LIBRARY_DIRS}
)

#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORKSIM_LIBRARIES
  ${RWS_SANDBOX}
  ${RWSIM_BULLET_LIBRARY}
  rwsim
  ${RWSIM_ODE_LIBRARY}
  rwsim
  ${ROBWORK_LIBRARIES}
)
 
 
