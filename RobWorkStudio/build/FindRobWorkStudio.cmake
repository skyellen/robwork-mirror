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

# Check if RW_ROOT path are setup correctly
FIND_FILE(ROBWORKSTUDIO_FOUND RobWorkStudioSetup.cmake 
    ${RWSTUDIO_ROOT}/build 
  	"${ROBWORKSTUDIO_ROOT}/build/"
	"${RWS_ROOT}/build/"
	"../build/"
	"../RobWorkStudio/build/"
	"../../RobWorkStudio/build/"
	"c:/program files/RobWork/build/"
	"c:/programmer/RobWork/build/"
)

IF(NOT ROBWORKSTUDIO_FOUND)
 MESSAGE(SEND_ERROR "Path to RobWorkStudio root (RWSTUDIO_ROOT) is incorrectly setup! \nRWSTUDIO_ROOT == ${RWSTUDIO_ROOT}")
ENDIF()


# Add extra linker flags for linking with OpenGL on mac os computers
if (DEFINED APPLE)
  set(flags
    "-dylib_file /System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${flags}")
endif ()


# Find and setup Qt4.
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTDESIGNER 1)
SET(QT_USE_QTUITOOLS 1)
INCLUDE(${QT_USE_FILE})

#ADD_DEFINITIONS(-DQT_PLUGIN)

# Find and setup OpenGL.
FIND_PACKAGE(OpenGL REQUIRED)

IF (DEFINED MSVC)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D QT_NO_DEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -D QT_NO_DEBUG")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -D QT_NO_DEBUG")
ELSE ()
  IF(NOT DEFINED RWS_CXX_FLAGS)
    IF (CMAKE_COMPILER_IS_GNUCXX)
    	IF( CMAKE_BUILD_TYPE STREQUAL "Debug" )
	        SET(RWS_CXX_FLAGS_TMP "-DQT_DEBUG") 
	    ELSE ()
	    	SET(RWS_CXX_FLAGS_TMP "-DQT_NO_DEBUG")
	    ENDIF()
    ENDIF ()
ENDIF()
ENDIF ()

# Setup the libraries

IF (RWS_BUILD_WITH_SANDBOX)
  SET(RWS_SANDBOX_LIB rwstudio_sandbox)
  SET(RWS_HAVE_SANDBOX ON)
ELSE ()
  SET(RWS_HAVE_SANDBOX OFF) 
ENDIF ()

SET(ROBWORKSTUDIO_LIBRARIES
    ${RWS_SANDBOX_LIB}
    rwstudio
  	${QT_LIBRARIES}
)

IF(DEFINED MSVC)
  SET(RWS_LIBS_DIR "${RWSTUDIO_ROOT}/libs/")
ELSE()
  SET(RWS_LIBS_DIR "${RWSTUDIO_ROOT}/libs/${CMAKE_BUILD_TYPE}/")
ENDIF()

# Setup RobWorkStudio include and link directories
SET(ROBWORKSTUDIO_INCLUDE_DIR ${RWSTUDIO_ROOT}/src/)
SET(ROBWORKSTUDIO_LIBRARY_DIRS ${RWS_LIBS_DIR})
 
 
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