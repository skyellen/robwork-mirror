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

SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})

# Try and find the robwork root path by checking the standard paths
FIND_FILE(RW_ROOT_PATH_TEST RobWorkSetup.cmake 
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

ENABLE_LANGUAGE(CXX)
#ENABLE_LANGUAGE(Fortran)

# get the build configuration of the requested built type
INCLUDE(${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake)

MESSAGE(STATUS "RobWork VERSION: ${RW_BUILD_WITH_VERSION}")

####################################################################
# DEPENDENCIES - REQUIRED
# first we check the required dependencies
#

#
# some of the FIND_PACKAGE modules are located in the build directory 
#
SET(CMAKE_LIBRARY_PATH_TMP CMAKE_LIBRARY_PATH) 
LIST(APPEND CMAKE_LIBRARY_PATH ${RW_BUILD_WITH_LIBRARY_DIRS})

#MESSAGE("sfdjafjaf: ${CMAKE_LIBRARY_PATH}")
#
# We need the boost package and some of its components
#
IF(NOT DEFINED BOOST_ROOT OR NOT BOOST_ROOT)
    SET(BOOST_ROOT ${RW_BUILD_WITH_BOOST_ROOT})
ENDIF()
IF(NOT DEFINED BOOST_INCLUDEDIR OR NOT BOOST_INCLUDEDIR)
    SET(BOOST_INCLUDEDIR ${RW_BUILD_WITH_BOOST_INCLUDE_DIR})
ENDIF()
IF(NOT DEFINED BOOST_LIBRARYDIR OR NOT BOOST_LIBRARYDIR)
    SET(BOOST_LIBRARYDIR ${RW_BUILD_WITH_BOOST_LIBRARY_DIR})
ENDIF()

FIND_PACKAGE(Boost COMPONENTS thread filesystem system regex REQUIRED )

IF(NOT DEFINED WIN32)
  SET(BLA_STATIC ON)
ENDIF()
#SET(BLA_STATIC ON) # for some reason this creates an error in the system and find_library will not function correctly after this
#FIND_PACKAGE(BLAS REQUIRED)
#FIND_PACKAGE(LAPACK REQUIRED) # automatically detects BLAS
FIND_PACKAGE(BLASLAPACK REQUIRED)
#MESSAGE("LAPACK LIBRARIES: ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES}") 

####################################################################
# DEPENDENCIES - OPTIONAL
# these dependencies are optional, which is robwork is not necesarilly built
# with these modules
# 



#
# For some libs we need the opengl package, though it is OPTIONAL 
#
IF(RW_BUILD_WITH_OPENGL)
    FIND_PACKAGE(OpenGL)
    IF(OPENGL_FOUND)
        SET(RW_DRAWABLE_LIB "rw_drawable")
    ENDIF()
ENDIF()

#
# if robwork was build with xerces then we also need to link with it now
#
IF(RW_BUILD_WITH_XERCES)
    IF(NOT DEFINED XERCESC_INCLUDE_DIR OR NOT XERCESC_INCLUDE_DIR)
        SET(XERCESC_INCLUDE_DIR ${RW_BUILD_WITH_XERCES_INCLUDE_DIR})
    ENDIF()
    IF(NOT DEFINED XERCESC_LIB_DIR OR NOT XERCESC_LIB_DIR)
        SET(XERCESC_LIB_DIR ${RW_BUILD_WITH_XERCES_LIB_DIR})
    ENDIF()
    #MESSAGE(status "Xerces: ${XERCESC_LIB_DIR} ${XERCESC_INCLUDE_DIR}")
    FIND_PACKAGE(XercesC REQUIRED)
    IF( XERCESC_FOUND )
        MESSAGE(STATUS "RobWork: Xerces REQUIRED! FOUND!")
        SET(RW_HAVE_XERCES True)
    ELSE ()
        MESSAGE(SEND_ERROR "RobWork: Xerces REQUIRED! NOT FOUND! Check if XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly!")
    ENDIF ()
ELSE()
    MESSAGE(STATUS "RobWork: Xerces DISABLED! Not build into RobWork!")
ENDIF()


#
# If robwork was build with yaobi then we also need to link with it now
#
IF(RW_BUILD_WITH_YAOBI)   
    IF(NOT DEFINED YAOBI_INCLUDE_DIR OR NOT YAOBI_INCLUDE_DIR)
        SET(YAOBI_INCLUDE_DIR ${RW_BUILD_WITH_YAOBI_INCLUDE_DIR})
    ENDIF()

    FIND_PACKAGE(Yaobi REQUIRED)
    IF( YAOBI_FOUND )
        MESSAGE(STATUS "RobWork: Yaobi REQUIRED! FOUND!")
    ELSE ()
        MESSAGE(SEND_ERROR "RobWork: Yaobi REQUIRED! NOT FOUND! Try setting YAOBI_INCLUDE_DIR and YAOBI_LIB_DIR.")
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWork: Yaobi DISABLED! Not build into RobWork!")
    SET(YAOBI_INCLUDE_DIR "")
ENDIF()

#
# If robwork was build with pqp then we also need to link with it now
#


IF(RW_BUILD_WITH_PQP)
    IF(NOT DEFINED PQP_INCLUDE_DIR OR NOT PQP_INCLUDE_DIR)
        SET(PQP_INCLUDE_DIR ${RW_BUILD_WITH_PQP_INCLUDE_DIR})
    ENDIF()

    FIND_PACKAGE(PQP REQUIRED)
    IF( PQP_FOUND )
        MESSAGE(STATUS "RobWork: PQP REQUIRED! FOUND!")
    ELSE ()
        MESSAGE(SEND_ERROR "RobWork: PQP REQUIRED! NOT FOUND! Try setting PQP_INCLUDE_DIR and PQP_LIB_DIR.")
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWork: PQP DISABLED! Not build into RobWork!")   
    SET(PQP_INCLUDE_DIR "")
ENDIF()


#
# If robwork was build with Lua then we also need to link with it now or not use rw_lua
#
SET(RW_USE_RW_LUA True)
IF(RW_BUILD_WITH_LUA)
    # if user does not supply lua include dir then try to use that from robwork
    IF(NOT DEFINED LUA_INCLUDE_DIR OR NOT LUA_INCLUDE_DIR)
        SET(LUA_INCLUDE_DIR ${RW_BUILD_WITH_LUA_INCLUDE_DIR})
    ENDIF()
    
    MESSAGE(STATUS "RobWork: LUA ENABLED!")
    FIND_PACKAGE(Lua51 QUIET)
    IF( LUA51_FOUND )
        MESSAGE(STATUS "FOUND Lua!")
    ELSE ()
        SET(RW_USE_RW_LUA False)
        MESSAGE(STATUS "Lua NOT FOUND! Disabling use of rw_lua.")
    ENDIF ()

    IF(NOT DEFINED TOLUA_CMD OR NOT TOLUA_CMD)
        SET(TOLUA_CMD ${RW_BUILD_WITH_TOLUA_CMD})
    ENDIF()
    IF(NOT DEFINED TOLUA_INCLUDE_DIR OR NOT TOLUA_INCLUDE_DIR) 
        SET(TOLUA_INCLUDE_DIR ${RW_BUILD_WITH_TOLUA_INCLUDE_DIR})
    ENDIF()
    
    FIND_PACKAGE(Tolua++ QUIET)
    IF( TOLUA++_FOUND )
        MESSAGE(STATUS "FOUND Tolua!")
    ELSE ()
        SET(RW_USE_RW_LUA False)
        MESSAGE(STATUS "Tolua NOT FOUND! Disabling use of rw_lua.")
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWork: LUA DISABLED! Not build into RobWork!")   
    SET(LUA_INCLUDE_DIR "")
    SET(TOLUA_INCLUDE_DIR "")
    SET(RW_USE_RW_LUA False)
ENDIF()

IF(RW_USE_RW_LUA)
    SET(RW_LUA_LIBRARY rw_lua)    
ENDIF()


IF (RW_BUILD_WITH_SANDBOX)
    MESSAGE(STATUS "RobWork: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rw_sandbox")
ELSE ()
    MESSAGE(STATUS "RobWork: Sandbox DISABLED! Not build into RobWork!")    
ENDIF ()


#######################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Enable the RW_ASSERT() macro.
#
IF( RW_BUILD_WITH_RW_ASSERT )
    MESSAGE(STATUS "RobWork: RW_ASSERT enabled.")
    ADD_DEFINITIONS(-DRW_ENABLE_ASSERT)
ELSE ()
    MESSAGE(STATUS "RobWork: RW_ASSERT disabled.")
ENDIF ()

#
# Set extra compiler flags. The user should be able to change this
#
SET(RW_CXX_FLAGS ${RW_BUILD_WITH_CXX_FLAGS} 
    CACHE STRING "Change this to force using your own 
                  flags and not those of RobWork"
)
ADD_DEFINITIONS(${RW_CXX_FLAGS})
MESSAGE(STATUS "RobWork: Using CXX flags: ${RW_CXX_FLAGS}") 

#MESSAGE(" ${Boost_MAJOR_VERSION} ${Boost_MINOR_VERSION} ")
IF(${Boost_MINOR_VERSION} VERSION_LESS 41 ) 
    # proerty tree is not included in earlier versions 1.41 of boost
    # so we include it from our own
    SET(ADDITIONAL_BOOST_BINDINGS "${RW_ROOT}/ext/deprecated")
    MESSAGE(STATUS "RobWork: Boost ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION} found, no support for property_tree. Adding from ext!")   
ENDIF()



###########################################################################
# SETTING UP VARS
# here we setup the output variables
# 

#
# The include dirs
#
SET(ROBWORK_INCLUDE_DIR
    ${RW_ROOT}/ext
    ${ADDITIONAL_BOOST_BINDINGS}
    ${RW_ROOT}/src
    ${OPENGL_INCLUDE_DIR}   
    ${Boost_INCLUDE_DIR}
    ${XERCESC_INCLUDE_DIR}
    ${YAOBI_INCLUDE_DIR}
    ${PQP_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${TOLUA_INCLUDE_DIR}
)

#
# The library dirs
#
SET(ROBWORK_LIBRARY_DIRS
    ${Boost_LIBRARY_DIRS}
    ${XERCESC_LIBRARY_DIRS}
    ${YAOBI_LIBRARY_DIRS}
    ${PQP_LIBRARY_DIRS}
    ${LUA_LIBRARY_DIRS}
    ${TOLUA_LIBRARY_DIRS}
    "${RW_ROOT}/libs/${CMAKE_BUILD_TYPE}"
    ${RW_LIBRARY_OUT_DIR}
    ${RW_ARCHIVE_OUT_DIR}
)
#MESSAGE(STATUS "${ROBWORK_LIBRARY_DIRS}")


#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORK_LIBRARIES
  ${SANDBOX_LIB}
  "rw_control"
  "rw_algorithms"
  "rw_pathplanners"
  "rw_pathoptimization"
  "rw_task"
  ${RW_DRAWABLE_LIB}
  "rw_simulation"
  ${RW_LUA_LIBRARY}
  ${TOLUA_LIBRARIES}
  ${LUA_LIBRARIES}
  "rw_proximitystrategies"
  ${YAOBI_LIBRARIES}
  ${PQP_LIBRARIES}
  "rw"
  ${OPENGL_LIBRARIES}
  ${XERCESC_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LAPACK_LIBRARIES} 
  ${BLAS_LIBRARIES})

SET(ROBWORK_VERSION ${RW_BUILD_WITH_VERSION})
SET(CMAKE_LIBRARY_PATH CMAKE_LIBRARY_PATH_TMP)
#MARK_AS_ADVANCED(ROBWORK_LIBRARIES ROBWORK_LIBRARY_DIRS ROBWORK_INCLUDE_DIR)
