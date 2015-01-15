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

# if RWS_ROOT is defined then we use that, else we try finding it the traditional way
IF(RWS_ROOT)
    SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKSTUDIODEVEL_FOUND RobWorkStudioSetup.cmake 
        ${RWSIM_ROOT}/../RobWorkStudio/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RobWorkStudio_DIR ${RWSIM_ROOT}/../RobWorkStudio/cmake)    
    ENDIF()
ENDIF()
#MESSAGE("RWS_ROOT ${RobWorkStudio_DIR}")

# now include the correct setup stuff
#SET(RobWorkStudio_DIR ${CMAKE_CURRENT_LIST_DIR}/../cmake)
#MESSAGE("RobWork_DIR ${RobWork_DIR}")
FIND_PACKAGE(RobWorkStudio ${RobWorkStudio_FIND_VERSION} REQUIRED NO_MODULE)
