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
#  RWSTUDIO_ROOT             - If set this defines the root of ROBWORKSTUDIO if not set then it
#                              if possible be autodetected.
#
#  
#

# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Check if RWstudio_ROOT path are setup correctly
FIND_FILE(ROBWORKSTUDIO_FOUND RobWorkStudioSetup.cmake ${RWSTUDIO_ROOT}/build NO_DEFAULT_PATH)
IF(NOT ROBWORKSTUDIO_FOUND)
 MESSAGE(SEND_ERROR "RobWorkStudio: Path to RobWorkStudio root (RWSTUDIO_ROOT) is incorrectly setup! \nRWSTUDIO_ROOT == ${RWSTUDIO_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWorkStudio: ROOT dir: ${RWSTUDIO_ROOT}")

#
# Setup the default include and library dirs for RobWorkStudio
#
#INCLUDE("${RWSTUDIO_ROOT}/build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake")


 ####################################################################
# DEPENDENCIES - REQUIRED
# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
#
 
#Include default settings for constructing a robwork dependent project
SET(ROBWORK_ROOT ${RW_ROOT})
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)

STRING(COMPARE EQUAL "${ROBWORKSTUDIO_VERSION}" "${ROBWORK_VERSION}" COMPATIBLE_VERSION)
IF( NOT COMPATIBLE_VERSION )
    MESSAGE(SEND_ERROR "RobWorkStudio: Version of RobWork ${ROBWORK_VERSION} is incompatible with version of RobWorkStudio ${ROBWORKSTUDIO_VERSION}")
ENDIF()

# Find and setup OpenGL.
FIND_PACKAGE(OpenGL REQUIRED)

# And some extra packages for boost
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

# Find and setup Qt4.
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTDESIGNER 1)
SET(QT_USE_QTUITOOLS 1)
INCLUDE(${QT_USE_FILE})

 
 ####################################################################
# DEPENDENCIES - OPTIONAL
# these dependencies are optional, which is the user can switch off
# modules
  
# optional compilation of sandbox
IF (RWS_BUILD_SANDBOX)
    MESSAGE(STATUS "RobWorkStudio: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rwstudio_sandbox")
    SET(RWS_HAVE_SANDBOX true)
ELSE ()
    MESSAGE(STATUS "RobWorkStudio: Sandbox DISABLED!")    
ENDIF ()


# optional compilation of sandbox
SET(RWS_HAVE_LUA False)
OPTION(RWS_USE_LUA "Set when you want to use lua!" ${RWS_USE_LUA})
IF( RWS_USE_LUA )
    IF (RW_BUILD_WITH_LUA)
        MESSAGE(STATUS "RobWorkStudio: Lua ENABLED!")
        SET(RWS_LUA "rwstudio_lua")
        SET(RWS_HAVE_LUA True)
    ELSE ()
        MESSAGE(SEND_ERROR "RobWorkStudio: Lua DISABLED! - RobWork is NOT compiled with Lua support! Set RWS_USE_LUA=OFF")
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
# The compiler flags from RobWork is automatically set 
#
IF(NOT DEFINED RWS_CXX_FLAGS)
    IF (CMAKE_COMPILER_IS_GNUCXX)
    	IF( CMAKE_BUILD_TYPE STREQUAL "Debug" )
	        SET(RWS_CXX_FLAGS_TMP "-DQT_DEBUG") 
	    ELSE ()
	    	SET(RWS_CXX_FLAGS_TMP "-DQT_NO_DEBUG")
	    ENDIF()
    
      # Setup crucial MSVC flags, without these RobWork does not compile
      # TODO: hmm, is this right?
      # IF (DEFINED MSVC)
      #  SET(RWS_CXX_FLAGS_TMP "-DQT_NO_DEBUG")
      # ENDIF ()
    ENDIF ()
ENDIF()
SET(RWS_CXX_FLAGS ${RWS_CXX_FLAGS_TMP} 
    CACHE STRING "Change this to force using your own 
                  flags and not those of RobWorkSutdio"
)
ADD_DEFINITIONS(${RWS_CXX_FLAGS})
MESSAGE(STATUS "RobWorkStudio: Adding RWS CXX flags: ${RWS_CXX_FLAGS}") 
		 

# If we are using static linking then remember to 
IF (RWS_USE_STATIC_LINK_PLUGINS)
  MESSAGE(STATUS "RobWorkStudio: Using static linking of default plugins!")
ELSE ()
  MESSAGE(STATUS "RobWorkStudio: Using dynamic linking of default plugins!")
ENDIF ()

#
# Setting up macro for easilly adding rws plugin target 
#
MACRO (RWS_QT4_WRAP_UI outfiles )
QT4_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

FOREACH (it ${ui_files})
  GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
  GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
  SET(outfile ${CMAKE_CURRENT_SOURCE_DIR}/ui_${outfile}.h)
  ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
    COMMAND ${QT_UIC_EXECUTABLE}
    ARGS ${ui_options} -o ${outfile} ${infile}
    MAIN_DEPENDENCY ${infile})
  SET(${outfiles} ${${outfiles}} ${outfile})
ENDFOREACH (it)

ENDMACRO (RWS_QT4_WRAP_UI)
 
###########################################################################
# SETTING UP VARS
# here we setup the output variables
# 

# Setup RobWorkStudio include and link directories
SET(ROBWORKSTUDIO_INCLUDE_DIR ${RWSTUDIO_ROOT}/src/)
SET(ROBWORKSTUDIO_LIBRARY_DIRS ${RWS_LIBS_DIR})
#
# The include dirs
#
SET(ROBWORKSTUDIO_INCLUDE_DIR
    ${RWSTUDIO_ROOT}/src
    ${Boost_INCLUDE_DIR}
    ${ROBWORK_INCLUDE_DIR}
    ${RWSTUDIO_ROOT}/ext/qtpropertybrowser/src/
)

#
# The library dirs
#
SET(ROBWORKSTUDIO_LIBRARY_DIRS
    ${Boost_LIBRARY_DIRS}
    ${ROBWORK_LIBRARY_DIRS}
    ${RWS_LIBRARY_OUT_DIR} 
    ${RWS_ARCHIVE_OUT_DIR}
)
#MESSAGE("${ROBWORK_LIBRARY_DIRS}")
#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORKSTUDIO_LIBRARIES
  ${RWS_SANDBOX}
  ${RWS_LUA}
  "rwstudio"
  ${ROBWORK_LIBRARIES}
  qtpropertybrowser
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
 
 
