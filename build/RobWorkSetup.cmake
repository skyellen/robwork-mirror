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

# Check if RW_ROOT path are setup correctly
FIND_FILE(RW_ROOT_PATH_TEST RobWorkSetup.cmake ${RW_ROOT}/build NO_DEFAULT_PATH)
IF(NOT RW_ROOT_PATH_TEST)
    MESSAGE(SEND_ERROR "Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWork ROOT dir: ${RW_ROOT}")

# Setup the default include and library dirs for robwork
INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")
SET(ROBWORK_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR} ${RW_ARCHIVE_OUT_DIR})
SET(ROBWORK_INCLUDE_DIR
  ${RW_ROOT}/ext
  ${RW_ROOT}/src
  ${RW_ROOT}/ext/lua/src
  ${RW_ROOT}/ext/tolua/include
)

# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
INCLUDE(${RW_ROOT}/build/depends.cmake)
#MESSAGE(${ROBWORK_LIBRARY_DIRS})
#MESSAGE(${ROBWORK_INCLUDE_DIR})

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
  
  add_definitions(-D_HAS_ITERATOR_DEBUGGING=0)
  add_definitions(-D_SECURE_SCL=0)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
ENDIF ()

INCLUDE(${RW_ROOT}/build/link.cmake)

# Setup the Library List here. We need to make sure the correct order is maintained 
SET(ROBWORK_LIBRARIES
  ${SANDBOX_LIB}
  "rw_algorithms"
  "rw_pathplanners"
  "rw_pathoptimization"
  "rw_proximitystrategies"
  "rw_drawable"
  "rw_simulation"
  ${RW_LIBRARY_LIST}
  ${RW_COLLISION_DETECTION_LIBS}
  ${OPENGL_LIBRARIES}
  ${XERCESC_LIBRARIES}
  ${Boost_LIBRARIES}
)
