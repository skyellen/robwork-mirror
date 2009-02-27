# Find and sets up RobWork
# 
#  ROBWORK_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORK_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORK_LIBARY_DIRS - List of directories where libraries of RobWork are located. 
#  ROBWORK_FOUND       - True if RobWork was found. (not impl yet)

#
# Allow the syntax else (), endif (), etc.
#
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

SET(RW_POSTFIX "")
IF( ${CMAKE_BUILD_TYPE} STREQUAL "Debug" )
    SET(RW_POSTFIX "_d")
ENDIF ()

# Check if RW_ROOT path are setup correctly
FIND_FILE(RW_ROOT_PATH_TEST RobWorkSetup.cmake ${RW_ROOT}/build NO_DEFAULT_PATH)
IF(NOT RW_ROOT_PATH_TEST)
    MESSAGE(SEND_ERROR "Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWork ROOT dir: ${RW_ROOT}")

# Enable the RW_ASSERT() macro.
OPTION(RW_ENABLE_ASSERT "Enables RW_ASSERT macro: on|off" ${RW_ENABLE_ASSERT})
IF( RW_ENABLE_ASSERT )
    MESSAGE(STATUS "RW_ASSERT enabled.")
    ADD_DEFINITIONS(-DRW_ENABLE_ASSERT)
ELSE ()
    MESSAGE(STATUS "RW_ASSERT disabled.")
ENDIF ()

# include the build specific configuration of RobWork 
INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

# Set extra compiler flags.
IF(RW_ENABLE_CXX_FLAGS)
    IF (CMAKE_COMPILER_IS_GNUCXX)
      IF (DEFINED MINGW)
        set(RW_CXX_FLAGS_TMP "-Wall")
      else ()
        set(RW_CXX_FLAGS_TMP "-Wall -fPIC")
      endif ()
    ENDIF ()
    SET(RW_CXX_FLAGS ${RW_CXX_FLAGS_TMP} CACHE STRING "The CXX flags used in the compilation!")
    MESSAGE(STATUS "Using CXX flags: ${RW_CXX_FLAGS}") 
    ADD_DEFINITIONS(${RW_CXX_FLAGS})
ENDIF ()


# Setup crucial MSVC flags, without these RobWork does not compile
IF (DEFINED MSVC)
  # Remove the min()/max() macros or else RobWork won't compile.
  ADD_DEFINITIONS(-DNOMINMAX)

  # Without this define for boost-bindings we can't link with lapack.
  ADD_DEFINITIONS(-DBIND_FORTRAN_LOWERCASE_UNDERSCORE)
  ADD_DEFINITIONS(-D_HAS_ITERATOR_DEBUGGING=0)
ENDIF ()


# Check for all dependencies
INCLUDE(${RW_ROOT}/build/depends.cmake)

# Setup include dirs for robwork
SET(ROBWORK_INCLUDE_DIR
  ${RW_ROOT}/ext
  ${RW_ROOT}/src
  ${RW_ROOT}/ext/lua/src
  ${RW_ROOT}/ext/tolua/include
  ${Boost_INCLUDE_DIR}
)

# Test to find PQP and yaobi header files
INCLUDE(CheckIncludeFileCXX)

#link_directories(${RW_ROOT}/ext)

LIST(APPEND CMAKE_REQUIRED_INCLUDES ${ROBWORK_INCLUDE_DIR})

CHECK_INCLUDE_FILE_CXX("PQP/PQP.h" RW_HAVE_PQP)
CHECK_INCLUDE_FILE_CXX("yaobi/yaobi.h" RW_HAVE_YAOBI)

INCLUDE(${RW_ROOT}/build/link.cmake)

# Setup the LibraryList 
SET(ROBWORK_LIBRARIES
  ${SANDBOX_LIB}
  "rw_algorithms${RW_POSTFIX}"
  "rw_pathplanners${RW_POSTFIX}"
  "rw_pathoptimization${RW_POSTFIX}"
  "rw_proximitystrategies${RW_POSTFIX}"
  "rw_drawable${RW_POSTFIX}"
  "rw_simulation${RW_POSTFIX}"
  ${RW_LIBRARY_LIST}
  ${RW_COLLISION_DETECTION_LIBS}
  ${OPENGL_LIBRARIES}
  ${Boost_LIBRARIES}
)
