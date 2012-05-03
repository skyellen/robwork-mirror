#########################################################################################
######################## THIS FILE IS DEPRECATED ########################################
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
# Variables that are set and which describe the setup
#
#  RW_BUILD_WITH_OPENGL 
#  RW_BUILD_WITH_XERCES 
#  RW_BUILD_WITH_PQP 
#  RW_BUILD_WITH_YAOBI 
#  RW_BUILD_WITH_LUA 
#  RW_BUILD_WITH_TOLUA 
#  RW_BUILD_WITH_SANDBOX 
#
#
# Allow the syntax else (), endif (), etc.
#
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Get the compiler architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(AMD64 1)
ELSE()
	SET(AMD64 0)
ENDIF()

IF(DEFINED RW_ROOT)
	FILE(TO_CMAKE_PATH ${RW_ROOT} RW_ROOT)
ENDIF()
IF(DEFINED ROBWORK_ROOT)
	FILE(TO_CMAKE_PATH ${ROBWORK_ROOT} ROBWORK_ROOT)
	IF(NOT DEFINED RW_ROOT)
		SET(RW_ROOT "${ROBWORK_ROOT}")
	ENDIF()
ENDIF()

# now include the correct setup stuff
SET(RobWork_DIR ${CMAKE_CURRENT_LIST_DIR}/../cmake)
MESSAGE("RobWork_DIR ${RobWork_DIR}")
FIND_PACKAGE(RobWork ${RobWork_FIND_VERSION} REQUIRED NO_MODULE)
