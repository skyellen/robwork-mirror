# Find and sets up RobWorkStudio. 
# 
#
#
#
#  ROBWORKSTUDIO_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORKSTUDIO_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORKSTUDIO_LIBRARY_DIRS - List of directories where libraries of RobWork are located. 
#  ROBWORKSTUDIO_FOUND       - True if RobWork was found. (not impl yet)
#
#  RWS_ROOT             - If set this defines the root of ROBWORKSTUDIO if not set then it
#                              if possible be autodetected.
#
#  
#

# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

IF( RWSTUDIO_ROOT )
    SET(RWS_ROOT ${RWSTUDIO_ROOT})
ENDIF()

# Check if RWstudio_ROOT path are setup correctly
FIND_FILE(ROBWORKSTUDIO_FOUND RobWorkStudioSetup.cmake ${RWSTUDIO_ROOT}/cmake ${RWS_ROOT}/cmake NO_DEFAULT_PATH)
IF(NOT ROBWORKSTUDIO_FOUND)
 MESSAGE(SEND_ERROR "RobWorkStudio: Path to RobWorkStudio root (RWSTUDIO_ROOT) is incorrectly setup! \nRWSTUDIO_ROOT == ${RWSTUDIO_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWorkStudio: ROOT dir: ${RWS_ROOT}")

#
# Setup the default include and library dirs for RobWorkStudio
#
#INCLUDE("${RWSTUDIO_ROOT}/cmake/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake")


 ####################################################################
# DEPENDENCIES - REQUIRED
# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
#
 
#Include default settings for constructing a robwork dependent project
#SET(ROBWORK_ROOT ${RW_ROOT})
#SET(CMAKE_MODULE_PATH ${RW_ROOT}/cmake ${CMAKE_MODULE_PATH})
#SET(RobWork_DIR ${RW_ROOT}/cmake)
#FIND_PACKAGE(RobWork ${ROBWORKSTUDIO_VERSION_MAJOR}.${ROBWORKSTUDIO_VERSION_MINOR}.${ROBWORKSTUDIO_VERSION_PATCH})

#STRING(COMPARE EQUAL "${ROBWORKSTUDIO_VERSION}" "${ROBWORK_VERSION}" COMPATIBLE_VERSION)
#IF( NOT COMPATIBLE_VERSION )
#    MESSAGE(SEND_ERROR "RobWorkStudio: Version of RobWork ${ROBWORK_VERSION} is incompatible with version of RobWorkStudio ${ROBWORKSTUDIO_VERSION}")
#ENDIF()

# Find and setup OpenGL.
FIND_PACKAGE(OpenGL REQUIRED)

# And some extra packages for boost
UNSET(Boost_USE_STATIC_LIBS)
UNSET(Boost_FIND_QUIETLY)
IF(DEFINED UNIX)
  FIND_PACKAGE(Boost REQUIRED program_options)
ELSEIF(DEFINED WIN32)
  SET(Boost_USE_STATIC_LIBS ON)
  FIND_PACKAGE(Boost COMPONENTS program_options)
  # If static libraries for Windows were not found, try searching again for the shared ones
  IF(NOT Boost_PROGRAM_OPTIONS_FOUND)
    SET(Boost_USE_STATIC_LIBS OFF)
    FIND_PACKAGE(Boost REQUIRED program_options)
  ENDIF()
ENDIF()

# Find and setup Qt.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
FIND_PACKAGE(Qt5Core QUIET)
FIND_PACKAGE(Qt5Gui QUIET)
FIND_PACKAGE(Qt5Widgets QUIET)
FIND_PACKAGE(Qt5OpenGL QUIET)
IF(Qt5Core_FOUND AND Qt5Gui_FOUND AND Qt5Widgets_FOUND AND Qt5OpenGL_FOUND)
	get_target_property(QT_UIC_EXECUTABLE Qt5::uic LOCATION)
	SET(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
	SET(QT_INCLUDES ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${QT_INCLUDES})
	MESSAGE(STATUS "RobWorkStudio: Using Qt5.")
	SET(RWS_USE_QT5 ON)
ELSE()
	MESSAGE(STATUS "RobWorkStudio: One or more Qt5 modules not found:")
	IF(Qt5Core_FOUND)
		MESSAGE(STATUS "RobWorkStudio: - Qt5Core found.")
	ELSE()
		MESSAGE(STATUS "RobWorkStudio: - Qt5Core NOT found. Please set Qt5Core_DIR to find.")
	ENDIF()
	IF(Qt5Gui_FOUND)
		MESSAGE(STATUS "RobWorkStudio: - Qt5Gui found.")
	ELSE()
		MESSAGE(STATUS "RobWorkStudio: - Qt5Gui NOT found. Please set Qt5Gui_DIR to find.")
	ENDIF()
	IF(Qt5Widgets_FOUND)
		MESSAGE(STATUS "RobWorkStudio: - Qt5Widgets found.")
	ELSE()
		MESSAGE(STATUS "RobWorkStudio: - Qt5Widgets NOT found. Please set Qt5Widgets_DIR to find.")
	ENDIF()
	IF(Qt5OpenGL_FOUND)
		MESSAGE(STATUS "RobWorkStudio: - Qt5OpenGL found.")
	ELSE()
		MESSAGE(STATUS "RobWorkStudio: - Qt5OpenGL NOT found. Please set Qt5OpenGL_DIR to find.")
	ENDIF()
	FIND_PACKAGE(Qt4 QUIET)
	IF(Qt4_FOUND)
		SET(QT_USE_QTOPENGL 1)
		SET(QT_USE_QTDESIGNER 1)
		SET(QT_USE_QTUITOOLS 1)
		INCLUDE(${QT_USE_FILE})
		MESSAGE(STATUS "RobWorkStudio: Using Qt4.")
	ELSE()
		MESSAGE(FATAL_ERROR "RobWorkStudio: Could NOT find Qt4 or Qt5. Please set the Qt5 directories, or alternatively Qt4_DIR for Qt4.")
	ENDIF()
ENDIF()

 ####################################################################
# DEPENDENCIES - OPTIONAL
# these dependencies are optional, which is the user can switch off
# modules
  
# optional compilation of sandbox
IF (RWS_BUILD_SANDBOX)
    MESSAGE(STATUS "RobWorkStudio: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rws_sandbox")
    SET(RWS_HAVE_SANDBOX true)
ELSE ()
    MESSAGE(STATUS "RobWorkStudio: Sandbox DISABLED!")    
ENDIF ()

# Check if SWIG is available
IF(RW_BUILD_WITH_SWIG AND NOT DEFINED SWIG_EXECUTABLE)
    SET(SWIG_EXECUTABLE ${RW_BUILD_WITH_SWIG_CMD})
ENDIF()
FIND_PACKAGE(SWIG 1.3 QUIET)
IF(SWIG_FOUND)
    MESSAGE(STATUS "RobWorkStudio: SWIG ${SWIG_VERSION} found!")
ELSE()
    MESSAGE(STATUS "RobWorkStudio: SWIG not found!")
ENDIF()

# optional compilation of LUA interface
include(CMakeDependentOption)
SET(RWS_HAVE_LUA False)
CMAKE_DEPENDENT_OPTION(RWS_DISABLE_LUA "Set when you want to disable lua!" OFF "RW_BUILD_WITH_LUA AND SWIG_FOUND" ON)
IF( NOT RWS_DISABLE_LUA )
    IF(NOT SWIG_FOUND)
        MESSAGE(STATUS "RobWorkStudio: Lua DISABLED! - SWIG was not found!")
        SET(RWS_HAVE_LUA False)
    ELSEIF (RW_BUILD_WITH_LUA)
        MESSAGE(STATUS "RobWorkStudio: Lua ENABLED!")
        SET(RWS_LUA "rws_lua_s;rws_luaeditor")
        SET(RWS_HAVE_LUA True)
    ELSE ()
        MESSAGE(STATUS "RobWorkStudio: Lua DISABLED! - RobWork is NOT compiled with Lua support!")
        SET(RWS_HAVE_LUA False)
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWorkStudio: Lua DISABLED!")
ENDIF()

#######################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Set extra compiler flags. The user should be able to change this. 
# The compiler flags from RobWork are automatically set 
#
RW_IS_RELEASE(IS_RELEASE)

IF(NOT DEFINED RWS_CXX_FLAGS)
	SET(RWS_CXX_FLAGS "${RW_BUILD_WITH_CXX_FLAGS} ${RWS_CXX_FLAGS_TMP}" 
		CACHE STRING "Change this to force using your own flags and not those of RobWorkSutdio"
	)
ENDIF()

IF(NOT DEFINED RWS_DEFINITIONS)
	IF( ${IS_RELEASE} )
		SET(RWS_DEFINITIONS_TMP "-DQT_NO_DEBUG")
	ELSE ()
		SET(RWS_DEFINITIONS_TMP "-DQT_DEBUG") 
	ENDIF()
  
	SET(RWS_DEFINITIONS "${RW_BUILD_WITH_DEFINITIONS};${RWS_DEFINITIONS_TMP}" 
		CACHE STRING "Change this to force using your own definitions and not those of RobWorkSutdio"
	)
ENDIF()

ADD_DEFINITIONS(${RWS_DEFINITIONS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RWS_CXX_FLAGS}")
MESSAGE(STATUS "RobWorkStudio: Adding RWS CXX flags: ${RWS_CXX_FLAGS}")
MESSAGE(STATUS "RobWorkStudio: Addubg RWS definitions: ${RWS_DEFINITIONS}")

#
# Set extra linker flags. The user should be able to change this. 
# The linker flags from RobWork are automatically set.
#
IF(DEFINED RWS_LINKER_FLAGS)
	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RWS_LINKER_FLAGS}" CACHE STRING "" FORCE)
	IF(WIN32)
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RWS_LINKER_FLAGS}" CACHE STRING "" FORCE)
	ENDIF()
	
	MESSAGE(STATUS "RobWorkStudio: Adding RWS linker flags: ${RWS_LINKER_FLAGS}")
ENDIF()
#MESSAGE(STATUS "${RW_BUILD_WITH_CXX_FLAGS}")
# If we are using static linking
IF (RWS_USE_STATIC_LINK_PLUGINS)
  MESSAGE(STATUS "RobWorkStudio: Using static linking of default plugins!")
ELSE ()
  MESSAGE(STATUS "RobWorkStudio: Using dynamic linking of default plugins!")
ENDIF ()

 
###########################################################################
# SETTING UP VARS
# here we setup the output variables
# 

# Setup RobWorkStudio include and link directories
SET(ROBWORKSTUDIO_INCLUDE_DIR ${RWS_ROOT}/src/)
SET(ROBWORKSTUDIO_LIBRARY_DIRS ${RWS_LIBS_DIR})
#
# The include dirs
#
SET(ROBWORKSTUDIO_INCLUDE_DIR
    ${RWS_ROOT}/src
    ${Boost_INCLUDE_DIR}
    ${ROBWORK_INCLUDE_DIRS}
    ${RWS_ROOT}/ext/qtpropertybrowser/src/
)

#
# The library dirs
#
SET(ROBWORKSTUDIO_LIBRARY_DIRS
    ${Boost_LIBRARY_DIRS}
    ${ROBWORK_LIBRARY_DIRS}
    ${RWS_ROOT}/libs/${RWS_BUILD_TYPE}
)



#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORKSTUDIO_LIBRARIES
  ${RWS_SANDBOX}
  ${RWS_LUA}
  rws
  qtpropertybrowser
  ${ROBWORK_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
 
 
