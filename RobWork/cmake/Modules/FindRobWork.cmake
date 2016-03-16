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
#  RW_BUILD_WITH_ASSIMP 
#  RW_BUILD_WITH_ZLIB 
#  RW_BUILD_WITH_MINIZIP 
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
# test if module path is setup correctly
UNSET(RW_IN_MODULE_PATH_FOUND)
FIND_FILE(RW_IN_MODULE_PATH_FOUND RobWorkSetup.cmake PATHS ${CMAKE_MODULE_PATH} NO_DEFAULT_PATH)
#MESSAGE("RW_IN_MODULE_PATH_FOUND: ${RW_IN_MODULE_PATH_FOUND}")
IF(NOT RW_IN_MODULE_PATH_FOUND ) 
    MESSAGE("SETTING module path")
    SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
ENDIF()

# Try and find the robwork root path by checking the standard paths
FIND_FILE(RW_ROOT_PATH_TEST RobWorkSetup.cmake 
	"${ROBWORK_ROOT}/cmake/"
	"${RW_ROOT}/cmake/"
	"../cmake/"
	"../RobWork/cmake/"
	"../../RobWork/cmake/"
	"c:/program files/RobWork/cmake/"
	"c:/programmer/RobWork/cmake/"
)

IF(NOT RW_ROOT_PATH_TEST)
    MESSAGE(FATAL_ERROR "Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()

IF((DEFINED ROBWORK_INCLUDE_DIR) AND (DEFINED ROBWORK_LIBRARY_DIRS) AND (DEFINED ROBWORK_LIBRARIES) )
# Everything has alleready been loaded
#MESSAGE("(DEFINED ROBWORK_INCLUDE_DIR) AND (DEFINED ROBWORK_LIBRARY_DIRS) AND (DEFINED ROBWORK_LIBRARIES)")
    
ELSE ()

#MESSAGE(STATUS "RobWork Path: ${ROBWORK_ROOT}")

GET_FILENAME_COMPONENT(ROBWORK_ROOT_TMP  ${RW_ROOT_PATH_TEST} PATH)
SET(ROBWORK_ROOT "${ROBWORK_ROOT_TMP}/../")
SET(RW_ROOT ${ROBWORK_ROOT})
ENABLE_LANGUAGE(CXX)
#ENABLE_LANGUAGE(Fortran)

# make sure the build type is specified
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
ENDIF ()

# get the build configuration of the requested built type
IF(EXISTS ${RW_ROOT}/cmake/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake)
	INCLUDE(${RW_ROOT}/cmake/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake)
ELSE()
	INCLUDE(${RW_ROOT}/cmake/RobWorkBuildConfig.cmake)
ENDIF()


if( DEFINED RobWork_FIND_VERSION_MAJOR)
if(NOT (${RobWork_FIND_VERSION_MAJOR} EQUAL ${RW_BUILD_WITH_VERSION_MAJOR}) )
  MESSAGE(SEND_ERROR "The Version of RobWork is Not Correct!") 
endIF()
endIF()
if( RobWork_FIND_VERSION_MINOR)
if( NOT (${RobWork_FIND_VERSION_MINOR} EQUAL ${RW_BUILD_WITH_VERSION_MINOR}) )
  MESSAGE(SEND_ERROR "The Version of RobWork is Not Correct!") 
endIF()
endIF()

#MESSAGE(STATUS "RobWork VERSION: ${RW_BUILD_WITH_VERSION}")

####################################################################
# DEPENDENCIES - REQUIRED
# first we check the required dependencies
#

#
# some of the FIND_PACKAGE modules are located in the build directory 
#
SET(CMAKE_LIBRARY_PATH_TMP ${CMAKE_LIBRARY_PATH}) 
LIST(APPEND CMAKE_LIBRARY_PATH ${RW_BUILD_WITH_LIBRARY_DIRS})

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

#
# We need the boost package and some of its components.
# Test libraries are optional and can be compiled from header instead.
#

UNSET(Boost_USE_STATIC_LIBS)
UNSET(Boost_FIND_QUIETLY)
SET(Boost_LIBRARIES_TMP "")
IF(DEFINED UNIX)
  #SET(Boost_USE_STATIC_LIBS ON)
  FIND_PACKAGE(Boost REQUIRED filesystem regex serialization system thread program_options)
  SET(Boost_LIBRARIES_TMP ${Boost_LIBRARIES_TMP} ${Boost_LIBRARIES})  
  # Test libraries are optional
  SET(Boost_FIND_QUIETLY TRUE)

  # On Mac OS only the header only version of boost unit test seems to work for now, needs further investigation
  IF(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
    SET(Boost_LIBRARIES_TMP ${Boost_LIBRARIES_TMP} ${Boost_LIBRARIES})
  ENDIF(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  
  IF(NOT Boost_TEST_EXEC_MONITOR_FOUND OR NOT Boost_UNIT_TEST_FRAMEWORK_FOUND)
    # header only
    #SET(Boost_USE_STATIC_LIBS OFF)
    #FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
    SET(RW_USE_BOOST_STATIC_TEST_LIBS off)
  ELSE()
    # libraries found 
    SET(RW_USE_BOOST_STATIC_TEST_LIBS on)  
  ENDIF()
  
ELSEIF(DEFINED WIN32)
  SET(Boost_USE_STATIC_LIBS ON)
  FIND_PACKAGE(Boost COMPONENTS filesystem regex serialization system thread program_options)
  SET(Boost_LIBRARIES_TMP ${Boost_LIBRARIES_TMP} ${Boost_LIBRARIES})
  # If static libraries for Windows were not found, try searching again for the shared ones
  IF(NOT Boost_FILESYSTEM_FOUND OR NOT Boost_REGEX_FOUND OR NOT Boost_SERIALIZATION_FOUND OR
     NOT Boost_SYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    SET(Boost_USE_STATIC_LIBS OFF)
    FIND_PACKAGE(Boost REQUIRED filesystem regex serialization system thread program_options)
    SET(Boost_LIBRARIES_TMP ${Boost_LIBRARIES_TMP} ${Boost_LIBRARIES})
  ENDIF()
  
  # Test libraries are optional
  SET(Boost_USE_STATIC_LIBS ON)
  SET(Boost_FIND_QUIETLY TRUE)
  FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
  SET(Boost_LIBRARIES_TMP ${Boost_LIBRARIES_TMP} ${Boost_LIBRARIES})
  # If static libraries for Windows were not found, try searching again for the shared ones
  IF(NOT Boost_TEST_EXEC_MONITOR_FOUND OR NOT Boost_UNIT_TEST_FRAMEWORK_FOUND)
    #SET(Boost_USE_STATIC_LIBS OFF)
    #FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
    SET(RW_USE_BOOST_STATIC_TEST_LIBS off)
  ELSE()
    # libraries found 
    SET(RW_USE_BOOST_STATIC_TEST_LIBS on)  
    
  ENDIF()
ENDIF()
SET(Boost_LIBRARIES ${Boost_LIBRARIES_TMP})

# Print test libraries status
IF(Boost_TEST_EXEC_MONITOR_FOUND AND Boost_UNIT_TEST_FRAMEWORK_FOUND)
  #MESSAGE(STATUS "Found additional Boost libraries:")
	#MESSAGE(STATUS "  test_exec_monitor")
	#MESSAGE(STATUS "  unit_test_framework")
ELSE()
  # Set necessary directory for disabling linking with test libraries for MSVC
	IF(DEFINED MSVC)
		SET(BOOST_TEST_NO_LIB TRUE)
  ELSE()
    SET(BOOST_TEST_NO_LIB FALSE)
	ENDIF()
ENDIF()


IF(NOT DEFINED WIN32)
  SET(BLA_STATIC ON)
ENDIF()
FIND_PACKAGE(BLASLAPACK REQUIRED)

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
        SET(RW_DRAWABLE_LIB "rw_opengl")
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
    FIND_PACKAGE(XercesC REQUIRED)
    IF( XERCESC_FOUND )
        #MESSAGE(STATUS "RobWork: Xerces REQUIRED! FOUND!")
        SET(RW_HAVE_XERCES True)
    ELSE ()
        MESSAGE(SEND_ERROR "RobWork: Xerces REQUIRED! NOT FOUND! Check if XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly!")
    ENDIF ()
ELSE()
    #MESSAGE(STATUS "RobWork: Xerces DISABLED! Not built into RobWork!")
ENDIF()


#
# If robwork was build with yaobi then we also need to link with it now
#
IF(RW_BUILD_WITH_YAOBI)
    IF(NOT DEFINED YAOBI_INCLUDE_DIR OR NOT YAOBI_INCLUDE_DIR)
        SET(YAOBI_INCLUDE_DIR ${RW_BUILD_WITH_YAOBI_INCLUDE_DIR})
    ENDIF()
    IF(RW_BUILD_WITH_INTERNAL_YAOBI)
        SET(YAOBI_INCLUDE_DIR ${RW_BUILD_WITH_YAOBI_INCLUDE_DIR})
        SET(YAOBI_LIBRARIES yaobi)
        SET(YAOBI_LIBRARY_DIRS )
        
        #MESSAGE(STATUS "RobWork: Yaobi REQUIRED! Using RobWork Yaobi.")
    ELSE ()    
    
        FIND_PACKAGE(Yaobi REQUIRED)
        IF( YAOBI_FOUND )
            #MESSAGE(STATUS "RobWork: Yaobi REQUIRED! FOUND!")
        ELSE ()
            MESSAGE(SEND_ERROR "RobWork: Yaobi REQUIRED! NOT FOUND! Try setting YAOBI_INCLUDE_DIR and YAOBI_LIB_DIR.")
        ENDIF ()
    ENDIF()
ELSE ()
    #MESSAGE(STATUS "RobWork: Yaobi DISABLED! Not built into RobWork!")
    SET(YAOBI_INCLUDE_DIR "")
ENDIF()

#
# If robwork was build with pqp then we also need to link with it now
#


IF(RW_BUILD_WITH_PQP)
    IF(NOT DEFINED PQP_INCLUDE_DIR OR NOT PQP_INCLUDE_DIR)
        SET(PQP_INCLUDE_DIR ${RW_BUILD_WITH_PQP_INCLUDE_DIR})
    ENDIF()
    
    IF(RW_BUILD_WITH_INTERNAL_PQP)
        SET(PQP_INCLUDE_DIR ${RW_BUILD_WITH_PQP_INCLUDE_DIR})
        SET(PQP_LIBRARIES pqp)
        SET(PQP_LIBRARY_DIRS )
        
        #MESSAGE(STATUS "RobWork: PQP REQUIRED! Using RobWork PQP.")
    ELSE ()    
        FIND_PACKAGE(PQP REQUIRED)
        IF( PQP_FOUND )
            #MESSAGE(STATUS "RobWork: PQP REQUIRED! FOUND!")
        ELSE ()
            MESSAGE(SEND_ERROR "RobWork: PQP REQUIRED! NOT FOUND! Try setting PQP_INCLUDE_DIR and PQP_LIB_DIR.")
        ENDIF ()
    ENDIF()
ELSE ()
    #MESSAGE(STATUS "RobWork: PQP DISABLED! Not built into RobWork!")   
    SET(PQP_INCLUDE_DIR "")
ENDIF()


#
# If robwork was build with Lua then we also need to link with it now or not use rw_lua
#
SET(RW_USE_RW_LUA True)
IF(RW_BUILD_WITH_LUA AND RW_BUILD_WITH_SWIG)
    # if user does not supply lua include dir then try to use that from robwork
    IF(NOT DEFINED LUA_INCLUDE_DIR OR NOT LUA_INCLUDE_DIR)
        SET(LUA_INCLUDE_DIR ${RW_BUILD_WITH_LUA_INCLUDE_DIR})
    ENDIF()
    
    IF(RW_BUILD_WITH_INTERNAL_LUA)
        SET(LUA_INCLUDE_DIR ${RW_BUILD_WITH_LUA_INCLUDE_DIR})
        SET(LUA_LIBRARIES lua51)
        SET(LUA_LIBRARY_DIRS )        
        #MESSAGE(STATUS "RobWork: Lua ENABLED! Using RobWork Lua.")
    ELSE ()
        #MESSAGE(STATUS "RobWork: LUA ENABLED!")
        FIND_PACKAGE(Lua51 QUIET)
        IF( LUA51_FOUND )
            #MESSAGE(STATUS "FOUND Lua!")
        ELSE ()
            SET(RW_USE_RW_LUA False)
            #MESSAGE(STATUS "Lua NOT FOUND! Disabling use of rw_lua.")
        ENDIF ()
    ENDIF()
    
    #IF(NOT DEFINED TOLUA_CMD OR NOT TOLUA_CMD)
    #    SET(TOLUA_CMD ${RW_BUILD_WITH_TOLUA_CMD})
    #ENDIF()
    #IF(NOT DEFINED TOLUA_INCLUDE_DIR OR NOT TOLUA_INCLUDE_DIR) 
    #    SET(TOLUA_INCLUDE_DIR ${RW_BUILD_WITH_TOLUA_INCLUDE_DIR})
    #ENDIF()
    IF( NOT SWIG_EXECUTABLE ) 
        SET(SWIG_EXECUTABLE ${RW_BUILD_WITH_SWIG_CMD})
    ENDIF()
    FIND_PACKAGE(SWIG QUIET 1.3)
    IF( SWIG_FOUND AND RW_USE_RW_LUA)
        #MESSAGE(STATUS "RobWork: Swig ENABLED! Using Swig.")
    ELSE ()
        SET(RW_USE_RW_LUA False)
        MESSAGE(STATUS "Swig NOT FOUND! Disabling use of rw_lua.")
    ENDIF()
    
    #IF(RW_BUILD_WITH_INTERNAL_TOLUA)
	#FIND_PACKAGE(Tolua++ QUIET)
    #    SET(TOLUA_INCLUDE_DIR ${RW_BUILD_WITH_TOLUA_INCLUDE_DIR})
    #    SET(TOLUA_LIBRARIES tolua51)
    #    SET(TOLUA_LIBRARY_DIRS )
    #    MESSAGE(STATUS "RobWork: Tolua ENABLED! Using RobWork Tolua.")
    #ELSE ()
    #    FIND_PACKAGE(Tolua++ QUIET)
    #    IF( TOLUA++_FOUND )
    #        MESSAGE(STATUS "FOUND Tolua!")
    #    ELSE ()
    #        SET(RW_USE_RW_LUA False)
    #        MESSAGE(STATUS "Tolua NOT FOUND! Disabling use of rw_lua.")
    #    ENDIF ()
    #ENDIF()
ELSE ()
    #MESSAGE(STATUS "RobWork: LUA DISABLED! Lua and Swig was not built into RobWork!")   
    SET(LUA_INCLUDE_DIR "")
    SET(RW_USE_RW_LUA False)
ENDIF()

IF(RW_USE_RW_LUA)
    SET(RW_LUA_LIBRARY "rw_lua")
ENDIF()


IF (RW_BUILD_WITH_SANDBOX)
    #MESSAGE(STATUS "RobWork: Sandbox ENABLED!")
    SET(SANDBOX_LIB "rw_sandbox")
ELSE ()
    MESSAGE(STATUS "RobWork: Sandbox DISABLED! Not built into RobWork!")    
ENDIF ()

#
# If robwork was build with Assimp then we also need to link with it now
#

IF(RW_BUILD_WITH_ASSIMP)
    IF(NOT DEFINED ASSIMP_INCLUDE_DIR OR NOT ASSIMP_INCLUDE_DIR)
        SET(ASSIMP_INCLUDE_DIR ${RW_BUILD_WITH_ASSIMP_INCLUDE_DIR})
    ENDIF()
    
    IF(RW_BUILD_WITH_INTERNAL_ASSIMP)
        SET(ASSIMP_INCLUDE_DIR ${RW_BUILD_WITH_ASSIMP_INCLUDE_DIR})
        SET(ASSIMP_LIBRARIES rw_assimp)
        SET(ASSIMP_LIBRARY_DIRS )
    ELSE ()    
        FIND_PACKAGE(Assimp 3.0 REQUIRED)
        IF( NOT Assimp_FOUND )
            MESSAGE(SEND_ERROR "RobWork: Assimp REQUIRED! NOT FOUND! Try setting ASSIMP_INCLUDE_DIR and ASSIMP_LIB_DIR.")
        ENDIF ()
    ENDIF()
ELSE ()
    SET(ASSIMP_INCLUDE_DIR "")
ENDIF()

# If robwork was build with ZLIB then we also need to link with it now
#

IF(RW_BUILD_WITH_ZLIB)
    IF(NOT DEFINED ZLIB_INCLUDE_DIR OR NOT ZLIB_INCLUDE_DIR)
        SET(ZLIB_INCLUDE_DIR ${RW_BUILD_WITH_ZLIB_INCLUDE_DIR})
    ENDIF()
    
    IF(RW_BUILD_WITH_INTERNAL_ZLIB)
        SET(ZLIB_INCLUDE_DIR ${RW_BUILD_WITH_ZLIB_INCLUDE_DIR})
        SET(ZLIB_LIBRARIES rw_zlib)
        SET(ZLIB_LIBRARY_DIRS )
    ELSE ()    
        FIND_PACKAGE(ZLIB REQUIRED)
        IF( NOT ZLIB_FOUND )
            MESSAGE(SEND_ERROR "RobWork: ZLIB REQUIRED! NOT FOUND! Try setting ZLIB_INCLUDE_DIR and ZLIB_LIB_DIR.")
        ENDIF ()
    ENDIF()
ELSE ()
    SET(ZLIB_INCLUDE_DIR "")
ENDIF()


# If robwork was build with Minizip then we also need to link with it now
#

IF(RW_BUILD_WITH_MINIZIP)
    IF(NOT DEFINED MINIZIP_INCLUDE_DIR OR NOT MINIZIP_INCLUDE_DIR)
        SET(MINIZIP_INCLUDE_DIR ${RW_BUILD_WITH_MINIZIP_INCLUDE_DIR})
    ENDIF()
    
    IF(RW_BUILD_WITH_INTERNAL_MINIZIP)
        SET(MINIZIP_INCLUDE_DIR ${RW_BUILD_WITH_MINIZIP_INCLUDE_DIR})
        SET(MINIZIP_LIBRARIES rw_unzip)
        SET(MINIZIP_LIBRARY_DIRS )
    ELSE ()    
        FIND_PACKAGE(MINIZIP REQUIRED)
        IF( NOT MINIZIP_FOUND )
            MESSAGE(SEND_ERROR "RobWork: MINIZIP REQUIRED! NOT FOUND! Try setting MINIZIP_INCLUDE_DIR and MINIZIP_LIB_DIR.")
        ENDIF ()
    ENDIF()
ELSE ()
    SET(MINIZIP_INCLUDE_DIR "")
ENDIF()


#######################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Enable the RW_ASSERT() macro.
#
IF( RW_BUILD_WITH_RW_ASSERT )
    #MESSAGE(STATUS "RobWork: RW_ASSERT enabled.")
    ADD_DEFINITIONS(-DRW_ENABLE_ASSERT)
ELSE ()
    #MESSAGE(STATUS "RobWork: RW_ASSERT disabled.")
ENDIF ()

#
# Set extra compiler flags. The user should be able to change this
#
SET(RW_CXX_FLAGS ${RW_BUILD_WITH_CXX_FLAGS} 
    CACHE STRING "Change this to force using your own 
                  flags and not those of RobWork"
)
ADD_DEFINITIONS(${RW_CXX_FLAGS})
#MESSAGE(STATUS "RobWork: Using CXX flags: ${RW_CXX_FLAGS}")

#
# Set extra linker flags. The user should be able to change this
#
SET(RW_LINKER_FLAGS ${RW_BUILD_WITH_LINKER_FLAGS} 
    CACHE STRING "Change this to force using your own linker
                  flags and not those of RobWork"
)
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
IF(WIN32)
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
ENDIF()
#MESSAGE(STATUS "RobWork: Using linker flags: ${RW_LINKER_FLAGS}")

#MESSAGE(" ${Boost_MAJOR_VERSION} ${Boost_MINOR_VERSION} ")
IF(${Boost_MINOR_VERSION} VERSION_LESS 41 ) 
    # proerty tree is not included in earlier versions 1.41 of boost
    # so we include it from our own
    SET(ADDITIONAL_BOOST_BINDINGS "${RW_ROOT}/ext/deprecated")
    #MESSAGE(STATUS "RobWork: Boost ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION} found, no support for property_tree. Adding from ext!")   
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
    ${ZLIB_INCLUDE_DIRS}
    ${MINIZIP_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
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
    ${RW_ROOT}/libs
    ${RW_ROOT}/libs/${CMAKE_BUILD_TYPE}
    ${RW_LIBRARY_OUT_DIR}
    ${RW_ARCHIVE_OUT_DIR}
    ${ZLIB_LIBRARY_DIRS}
    ${MINIZIP_LIBRARY_DIRS}
    ${ASSIMP_LIBRARY_DIRS}
)


#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORK_LIBRARIES_TMP
    ${SANDBOX_LIB}
    rw_control
    rw_algorithms
    rw_pathplanners
    rw_pathoptimization
    rw_task
    rw_simulation
    ${RW_DRAWABLE_LIB}
    ${RW_LUA_LIBRARY}
    ${TOLUA_LIBRARIES}
    ${LUA_LIBRARIES}
    rw_proximitystrategies
    ${YAOBI_LIBRARIES}
    ${PQP_LIBRARIES}
    rw
    ${LAPACK_LIBRARIES} 
    ${BLAS_LIBRARIES}
    ${XERCESC_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${Boost_LIBRARIES}
    rw_qhull
    ${MINIZIP_LIBRARIES}
    ${ZLIB_LIBRARIES}
)

SET(ROBWORK_LIBRARIES)
FOREACH(l ${ROBWORK_LIBRARIES_TMP})
  UNSET(tmp CACHE)
  FIND_LIBRARY(tmp ${l} PATHS ${ROBWORK_LIBRARY_DIRS})
  IF(tmp)
    LIST(APPEND ROBWORK_LIBRARIES ${tmp})
  ELSE()
    LIST(APPEND ROBWORK_LIBRARIES ${l})
  ENDIF()
ENDFOREACH(l)

SET(ROBWORK_VERSION ${RW_BUILD_WITH_VERSION})
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_TMP})
#MARK_AS_ADVANCED(ROBWORK_LIBRARIES ROBWORK_LIBRARY_DIRS ROBWORK_INCLUDE_DIR)

LIST(APPEND ROBWORK_LIBRARIES ${OPENGL_LIBRARIES})

ENDIF()
