# Find and sets up RobWork
# 
#  ROBWORK_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORK_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORK_LIBRARY_DIRS - List of directories where libraries of RobWork are located. 
#  ROBWORK_FOUND       - True if RobWork was found. (not impl yet)
#  ROBWORK_CXX_FLAGS   - 
#  ROBWORK_ROOT	       - If not set it will be set as the root of RobWork

#  Variables that can be set to configure robwork
#  RW_USE_XERCES - On if you want to use Xerces for loading xml
#  RW_USE_YAOBI  - On if you use the Yaobi library for collision detection
#  RW_USE_PQP    - On if you use the PQP library for collision detection

#
# Allow the syntax else (), endif (), etc.
#
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

# Try and find the robwork root path by checking the standard paths
FIND_FILE(RW_ROOT_PATH_TEST FindRobWork.cmake 
	"${ROBWORK_ROOT}/build/"
	"${RW_ROOT}/build/"
	"../build/"
	"../RobWork/build/"
	"../../RobWork/build/"
	"c:/program files/RobWork/build/"
	"c:/programmer/RobWork/build/"
)

IF(NOT RW_ROOT_PATH_TEST)
    MESSAGE(FATAL_ERROR "Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()

GET_FILENAME_COMPONENT(ROBWORK_ROOT_TMP  ${RW_ROOT_PATH_TEST} PATH)
SET(ROBWORK_ROOT "${ROBWORK_ROOT_TMP}/../")
SET(RW_ROOT ${ROBWORK_ROOT})

MESSAGE(STATUS "RobWork Path: ${ROBWORK_ROOT}")

# Setup the default include and library dirs for robwork
SET(ROBWORK_INCLUDE_DIR 
	${RW_ROOT}/ext
	${RW_ROOT}/src
	${RW_ROOT}/ext/lua
    ${RW_ROOT}/ext/lua/src
    ${RW_ROOT}/ext/tolua
    ${RW_ROOT}/ext/tolua/include
)

# Output goes to bin/<CONFIG> and libs/<CONFIG> unless specified otherwise by the user.
SET(ROBWORK_LIBRARY_DIRS "${RW_ROOT}/libs/${CMAKE_BUILD_TYPE}/")

# get the build configuration of the requested built type
INCLUDE(${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake)

# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

# We need the boost package
FIND_PACKAGE(Boost COMPONENTS thread REQUIRED)
IF(Boost_FOUND)
	LIST(APPEND ROBWORK_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
	LIST(APPEND ROBWORK_INCLUDE_DIR ${Boost_INCLUDE_DIR})
ENDIF ()

# For some libs we need the opengl package, but its OPTIONAL
FIND_PACKAGE(OpenGL)

# For some of the xml parsing we need xerces
SET(RW_HAVE_XERCES False)
IF( RW_BUILD_WITH_XERCES )
	OPTION(RW_USE_XERCES "Set when you want to use xerces for xml loading" ON)
	IF(RW_USE_XERCES)
		FIND_PACKAGE(XercesC REQUIRED)
		IF( XERCESC_FOUND )
			MESSAGE(STATUS "Xerces ENABLED! FOUND!")
			SET(RW_HAVE_XERCES True)
			LIST(APPEND ROBWORK_LIBRARY_DIRS ${XERCESC_LIB_DIR})
			LIST(APPEND ROBWORK_INCLUDE_DIR ${XERCESC_INCLUDE_DIR})
		ELSE ()
			MESSAGE(STATUS "Xerces ENABLED! NOT FOUND! Check if XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly!")
		ENDIF ()
	ELSE ()
		MESSAGE(STATUS "Xerces DISABLED!")
	ENDIF ()
ENDIF ()


# If the user wants to use yaobi then search for it
IF( RW_BUILD_WITH_YAOBI )
	OPTION(RW_USE_YAOBI "Set when you want to use " ON)
	IF(RW_USE_YAOBI)
		# make sure that the include directory is correct 
		FIND_FILE(YAOBI_PATH "yaobi.h" PATHS ${YAOBI_INCLUDE_DIR} "${RW_ROOT}/ext/yaobi/")
		IF(NOT YAOBI_PATH)
			MESSAGE(SEND_ERROR "YAOBI_INCLUDE_DIR is not setup correctly!")
		ELSE ()
			SET(RW_HAVE_YAOBI true)
			LIST(APPEND ROBWORK_INCLUDE_DIR ${YAOBI_INCLUDE_DIR})
		ENDIF()
		# if the library is not found in YAOBI_LIBRARY_DIR then assume the default dir
		# in robwork ext is used
		LIST(APPEND ROBWORK_LIBRARY_DIRS ${YAOBI_LIBRARY_DIR})
		find_library(YAOBI_LIB "yaobi" PATHS ${ROBWORK_LIBRARY_DIRS})
		IF(NOT YAOBI_LIB)
			MESSAGE(SEND_ERROR "Yaobi Library not found! Please specify the Yaobi library dir with YAOBI_LIBRARY_DIR")
		ENDIF()
	ENDIF()
ENDIF()

# If the user wants to use PQP then search for it or use the default

IF( RW_BUILD_WITH_PQP )
	OPTION(RW_USE_PQP "" ON)
	IF(RW_USE_PQP)
		# make sure that the include directory is correct 
		FIND_FILE(PQP_PATH "PQP.h" PATHS ${PQP_INCLUDE_DIR} "${RW_ROOT}/ext/pqp/")
		IF(NOT PQP_PATH)
			MESSAGE(SEND_ERROR "PQP_INCLUDE_DIR is not setup correctly!")
		ELSE ()
			SET(RW_HAVE_PQP true)
			LIST(APPEND ROBWORK_INCLUDE_DIR ${PQP_INCLUDE_DIR})
		ENDIF()
		# if the library is not found in PQP_LIBRARY_DIR then assume the default dir
		# in robwork ext is used
		LIST(APPEND ROBWORK_LIBRARY_DIRS ${PQP_LIBRARY_DIR})
		find_library(PQP_LIB "pqp" PATHS ${ROBWORK_LIBRARY_DIRS})
		IF(NOT PQP_LIB)
			MESSAGE(SEND_ERROR "PQP Library not found! Please specify the PQP library dir with PQP_LIBRARY_DIR")
		ENDIF()
	ENDIF()
ENDIF()

# Enable the RW_ASSERT() macro.
OPTION(RW_ENABLE_ASSERT "Enables RW_ASSERT macro: on|off" ${RW_BUILD_WITH_RW_ASSERT} )
IF( RW_ENABLE_ASSERT )
	IF( RW_BUILD_WITH_RW_ASSERT )
		MESSAGE(STATUS "RW_ASSERT enabled.")	
	ELSE ()
		MESSAGE(STATUS "RW_ASSERT enabled. Though RobWork was not build with it enabled!")
	ENDIF()
    ADD_DEFINITIONS(-DRW_ENABLE_ASSERT)
ELSE ()
	IF( RW_BUILD_WITH_RW_ASSERT )
		MESSAGE(STATUS "RW_ASSERT disabled. Though RobWork was build with it enabled!")
	ELSE ()
		MESSAGE(STATUS "RW_ASSERT disabled.")	
	ENDIF()
ENDIF ()

# Set extra compiler flags.
IF (CMAKE_COMPILER_IS_GNUCXX)
  IF (DEFINED MINGW)
	SET(RW_CXX_FLAGS_TMP "-Wall")
  ELSE ()
	SET(RW_CXX_FLAGS_TMP "-Wall -fPIC")
  ENDIF ()
ENDIF ()
SET(ROBWORK_CXX_FLAGS ${RW_CXX_FLAGS_TMP} CACHE STRING "The ROBWORK CXX flags used in the compilation!")

# Setup crucial MSVC flags, without these RobWork does not compile
IF (DEFINED MSVC)
  # Remove the min()/max() macros or else RobWork won't compile.
  ADD_DEFINITIONS(-DNOMINMAX)

  # Without this define for boost-bindings we can't link with lapack.
  ADD_DEFINITIONS(-DBIND_FORTRAN_LOWERCASE_UNDERSCORE)
  ADD_DEFINITIONS(-D_HAS_ITERATOR_DEBUGGING=0)
  ADD_DEFINITIONS(-D_SECURE_SCL=0)
  ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ENDIF ()


# All mandatory libraries for linking with rw:
if (DEFINED MINGW)
  #set(RW_UBLAS_LIBRARY_NAMES lapack blas g2c)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED MSVC)
  set(RW_UBLAS_LIBRARY_NAMES lapack_win32 blas_win32)
elseif (DEFINED UNIX)
  set(RW_UBLAS_LIBRARY_NAMES lapack)
endif ()

#INCLUDE(${RW_ROOT}/config/link.cmake)

# Opengl
IF (NOT OPENGL_FOUND)
    MESSAGE("OpenGL not found! Libraries that depent on OpenGL will not be available!")
ELSE ()
    SET(RW_DRAWABLE_LIBS "rw_drawable" ${OPENGL_LIBRARIES})
	# Libraries for programs using rw_drawable.
	set(RW_DRAWABLE_LIBRARY_LIST
	  "rw_drawable"
	  ${RW_LIBRARY_LIST}
	  ${OPENGL_LIBRARIES}
	  )
ENDIF ()

IF (RW_BUILD_WITH_SANDBOX)
	IF(RW_USE_SANDBOX)
		MESSAGE(STATUS "RobWork Sandbox ENABLED!")
		SET(SANDBOX_LIB "rw_sandbox")
	ELSE ()
		MESSAGE(STATUS "RobWork Sandbox DISABLED!")    
	ENDIF ()
ENDIF ()


# Setup the Library List here. We need to make sure the correct order is maintained 
SET(ROBWORK_LIBRARIES
  ${SANDBOX_LIB} 
  "rw_lua"
  "rw_simulation"
  ${RW_DRAWABLE_LIBS}
  "rw_algorithms"
  "rw_task"
  "rw_pathplanners"
  "rw_pathoptimization"
  "rw_proximitystrategies"
  "rw"
  "tolualib"
  "lualib" 
  ${RW_UBLAS_LIBRARY_NAMES}  
  ${PQP_LIB}
  ${YAOBI_LIB}
  ${XERCESC_LIBRARIES}
  ${Boost_LIBRARIES}
)

#MARK_AS_ADVANCED(ROBWORK_LIBRARIES ROBWORK_LIBRARY_DIRS ROBWORK_INCLUDE_DIR)