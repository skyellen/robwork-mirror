###########################################################################
#
# This file is for setting up a build of RobWork. It should NOT be used
# for including RobWork in other projects, for that use FindRobWork.cmake 
# 
#  Requirements:
#  
#  RW_ROOT             - Must be set to the root dir of RobWork
#
#  Following is a list of variables that is set by this script: 
# 
#  ROBWORK_INCLUDE_DIR - Where to find robwork include sub-directory.
#  ROBWORK_LIBRARIES   - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#  ROBWORK_LIBARY_DIRS - List of directories where libraries of RobWork are located. 
#  
#

#
# Check if RW_ROOT path are setup correctly
#
FIND_FILE(RW_ROOT_PATH_TEST RobWorkSetup.cmake ${RW_ROOT}/cmake NO_DEFAULT_PATH)
IF(NOT RW_ROOT_PATH_TEST)
    MESSAGE(SEND_ERROR "RobWork: Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}")
ENDIF()
MESSAGE(STATUS "RobWork: ROOT - ${RW_ROOT}")

#
# Setup the default include and library dirs for robwork
#
#INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

####################################################################
# DEPENDENCIES - REQUIRED
# Check for all dependencies, this adds LIBRARY_DIRS and include dirs that 
# the configuration depends on
#

#
# some of the FIND_PACKAGE modules are located in the build directory 
#
SET(CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

#
# include the build specific configuration of RobWork
# 
#INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")
#LINK_DIRECTORIES(${RW_ARCHIVE_OUT_DIR} ${RW_LIBRARY_OUT_DIR})
#LIST(APPEND CMAKE_LIBRARY_PATH ${RW_LIBRARY_OUT_DIR})

#
# We need the Boost package and some of its components.
# Test libraries are optional and can be compiled from header instead.
#

UNSET(Boost_USE_STATIC_LIBS)
UNSET(Boost_FIND_QUIETLY)
IF(DEFINED UNIX)
  #SET(Boost_USE_STATIC_LIBS ON)
  FIND_PACKAGE(Boost REQUIRED filesystem regex serialization system thread program_options)  
  # Test libraries are optional
  SET(Boost_FIND_QUIETLY TRUE)
  FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
  
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
  # If static libraries for Windows were not found, try searching again for the shared ones
  IF(NOT Boost_FILESYSTEM_FOUND OR NOT Boost_REGEX_FOUND OR NOT Boost_SERIALIZATION_FOUND OR
     NOT Boost_SYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    SET(Boost_USE_STATIC_LIBS OFF)
    FIND_PACKAGE(Boost REQUIRED filesystem regex serialization system thread program_options)
  ENDIF()
  
  # Test libraries are optional
  SET(Boost_USE_STATIC_LIBS ON)
  SET(Boost_FIND_QUIETLY TRUE)
  FIND_PACKAGE(Boost COMPONENTS test_exec_monitor unit_test_framework)
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

MESSAGE(STATUS "RobWork: Boost version ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION} found!")

# Print test libraries status
IF(Boost_TEST_EXEC_MONITOR_FOUND AND Boost_UNIT_TEST_FRAMEWORK_FOUND)
  MESSAGE(STATUS "RobWork: Found additional Boost libraries: test_exec_monitor and unit_test_framework")
ELSE()
  # Set necessary directory for disabling linking with test libraries for MSVC
	IF(DEFINED MSVC)
		SET(BOOST_TEST_NO_LIB TRUE)
  ELSE()
    SET(BOOST_TEST_NO_LIB FALSE)
	ENDIF()
ENDIF()


#
# We depend on BLAS and Lapack. These depend on FORTRAN, so we enable that
#
ENABLE_LANGUAGE(CXX)
#ENABLE_LANGUAGE(Fortran)

IF(NOT DEFINED WIN32)
  SET(BLA_STATIC ON)
ENDIF()
#FIND_PACKAGE(BLAS REQUIRED)
#FIND_PACKAGE(LAPACK REQUIRED)
FIND_PACKAGE(BLASLAPACK REQUIRED)

SET(LAPACK_BLAS_LIBRARY_DIRS )
#get_filename_component(BLAS_LIBRARY_DIRS ${BLAS_LIBRARIES} PATH)
#get_filename_component(LAPACK_LIBRARY_DIRS ${LAPACK_LIBRARIES} PATH)

FOREACH(lib IN LISTS LAPACK_LIBRARIES BLAS_LIBRARIES)
    get_filename_component(TMP_DIR ${lib} PATH)
    LIST(APPEND LAPACK_BLAS_LIBRARY_DIRS ${TMP_DIR})
ENDFOREACH(lib)


####################################################################
# DEPENDENCIES - OPTIONAL
# these dependencies are optional, which is the user can switch off
# modules

#
# For some libs we need the opengl package, though it is OPTIONAL 
#
FIND_PACKAGE(OpenGL)
INCLUDE(CMakeDependentOption)
#
# For some of the xml parsing we need xerces, though it is OPTIONAL
#
SET(RW_HAVE_XERCES False)
FIND_PACKAGE(XercesC REQUIRED)
IF( XERCESC_FOUND )
    SET(RW_HAVE_XERCES True)
ELSE ()
    MESSAGE(SEND_ERROR "RobWork: Xerces NOT FOUND! Check if XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly!")
ENDIF ()

#
# If the user wants to use yaobi then search for it, OPTIONAL
#
SET(RW_HAVE_YAOBI False)
CMAKE_DEPENDENT_OPTION(RW_USE_YAOBI "Set to ON to include Yaobi support.
                Set YAOBI_INCLUDE_DIR and YAOBI_LIBRARY_DIR 
                to specify your own YAOBI else RobWork YAOBI will 
                be used!"
      ON "NOT RW_DISABLE_YAOBI" OFF)
IF(RW_USE_YAOBI)
    FIND_PACKAGE(Yaobi QUIET)
    IF( YAOBI_FOUND )
        MESSAGE(STATUS "RobWork: Yaobi ENABLED! FOUND!")
        SET(RW_HAVE_YAOBI True)
    ELSE ()
        SET(RW_ENABLE_INTERNAL_YAOBI_TARGET ON)
        MESSAGE(STATUS "RobWork: Yaobi ENABLED! NOT FOUND! Using RobWork native Yaobi.")
        SET(YAOBI_INCLUDE_DIR "${RW_ROOT}/ext/yaobi")
        SET(YAOBI_LIBRARIES "yaobi")
        SET(YAOBI_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
        SET(RW_HAVE_YAOBI True)
        
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWork: Yaobi DISABLED!")
    SET(YAOBI_INCLUDE_DIR "")
ENDIF()

#
# If the user wants to use PQP then search for it or use the default
#
SET(RW_HAVE_PQP False)
CMAKE_DEPENDENT_OPTION(RW_USE_PQP "Set to ON to include PQP support.
                Set PQP_INCLUDE_DIR and PQP_LIB_DIR 
                to specify your own PQP else RobWork PQP will 
                be used!" 
    ON "NOT RW_DISABLE_PQP" OFF
)
IF(RW_USE_PQP)
    FIND_PACKAGE(PQP QUIET)
    IF( PQP_FOUND )
        MESSAGE(STATUS "RobWork: PQP ENABLED! FOUND!")
        SET(RW_HAVE_PQP True)
    ELSE ()
        SET(RW_ENABLE_INTERNAL_PQP_TARGET ON)
        MESSAGE(STATUS "RobWork: PQP ENABLED! NOT FOUND! Using RobWork native PQP.")
        SET(PQP_INCLUDE_DIR "${RW_ROOT}/ext/PQP")
        SET(PQP_LIBRARIES "pqp")
        SET(PQP_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
        SET(RW_HAVE_PQP True)
    ENDIF ()
ELSE ()
    MESSAGE(STATUS "RobWork: PQP DISABLED!")   
    SET(PQP_INCLUDE_DIR "")
ENDIF()


#
# If the user wants to use LUA then search for it or use the default
#
SET(RW_HAVE_LUA False)
SET(RW_HAVE_SWIG False)

FIND_PACKAGE(SWIG 1.3 QUIET)
CMAKE_DEPENDENT_OPTION(RW_USE_LUA "Set to ON to include PQP support.
                Set PQP_INCLUDE_DIR and PQP_LIB_DIR 
                to specify your own PQP else RobWork PQP will 
                be used!" 
    ON "SWIG_FOUND;NOT RW_DISABLE_LUA" OFF)
    
IF(RW_USE_LUA)
    FIND_PACKAGE(Lua51 QUIET)
    IF( LUA51_FOUND )
        MESSAGE(STATUS "RobWork: External lua FOUND!")
    ELSE ()
        SET(RW_ENABLE_INTERNAL_LUA_TARGET ON)
        MESSAGE(STATUS "RobWork:  External lua NOT FOUND! Using RobWork native Lua.")
        SET(LUA_INCLUDE_DIR "${RW_ROOT}/ext/lua/src/")
        SET(LUA_LIBRARIES "lua51")
        SET(LUA_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
    ENDIF ()

    IF( SWIG_FOUND )
        MESSAGE(STATUS "RobWork: LUA ENABLED! Both SWIG and Lua FOUND!")
        SET(RW_HAVE_SWIG True)
        SET(RW_HAVE_LUA True)
        set(RW_LUA_LIBS rw_lua_s)
    ELSE ()
        SET(RW_HAVE_SWIG False)
        SET(RW_HAVE_LUA False)
        MESSAGE(SEND_ERROR "RobWork: Lua DISABLED! Since SWIG was NOT FOUND!")
    ENDIF ()
ELSE ()
    IF(SWIG_FOUND)
        
    ELSE()
        MESSAGE(STATUS "RobWork: LUA DISABLED! Swig not found!")
    ENDIF()
    SET(LUA_INCLUDE_DIR "")
    #SET(TOLUA_INCLUDE_DIR "")
ENDIF()
  
IF (RW_BUILD_SANDBOX)
    MESSAGE(STATUS "RobWork: RobWork Sandbox ENABLED!")
    SET(SANDBOX_LIB "rw_sandbox")
ELSE ()
    MESSAGE(STATUS "RobWork Sandbox DISABLED!")    
ENDIF ()


#######################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Enable the RW_ASSERT() macro.
#
OPTION(RW_ENABLE_ASSERT "Enables RW_ASSERT macro: on|off" ${RW_ENABLE_ASSERT})
IF( RW_ENABLE_ASSERT )
    MESSAGE(STATUS "RobWork: RW_ASSERT enabled.")
    ADD_DEFINITIONS(-DRW_ENABLE_ASSERT)
ELSE ()
    MESSAGE(STATUS "RobWork: RW_ASSERT disabled.")
ENDIF ()

RW_IS_RELEASE(IS_RELEASE)
#
# Set extra compiler flags. The user should be able to change this
#
IF(NOT DEFINED RW_CXX_FLAGS)    
    # GCC and MinGW
    IF (CMAKE_COMPILER_IS_GNUCXX)
      # Turn off annoying GCC warnings
      SET(RW_CXX_FLAGS_TMP "-Wall" "-Wno-strict-aliasing" "-Wno-deprecated" "-Wno-unused-function")
      
      IF(IS_RELEASE)
          LIST(APPEND RW_CXX_FLAGS_TMP "-DBOOST_DISABLE_ASSERTS")
      ENDIF() 
      
      # Necessary Linux-GCC flag
      IF(DEFINED UNIX)
        LIST(APPEND RW_CXX_FLAGS_TMP "-fPIC")
      ENDIF()
	  
      IF(DEFINED MINGW AND AMD64)
        LIST(APPEND RW_CXX_FLAGS_TMP "-DBOOST_USE_WINDOWS_H")
      ENDIF()
    ENDIF ()
    
    # Setup crucial MSVC flags, without these RobWork does not compile
    IF (DEFINED MSVC)
      SET(RW_CXX_FLAGS_TMP # Remove the min()/max() macros or else RobWork won't compile.
                           "-DNOMINMAX" 
                           # Without this define for boost-bindings we can't link with lapack.
                           "-DBIND_FORTRAN_LOWERCASE_UNDERSCORE"
                           "-D_SCL_SECURE_NO_WARNINGS"
                           "-D_CRT_SECURE_NO_WARNINGS"
                           "-D_CRT_SECURE_NO_DEPRECATE"
                           "-EHa"
                           "-bigobj"
      )
      
      IF(BOOST_TEST_NO_LIB)
        LIST(APPEND RW_CXX_FLAGS_TMP "-DBOOST_TEST_NO_LIB")
      ENDIF()

      # Current issues addressed for MSVC 64 bit:
      # 	- MSVC 64-bit does not support __asm keyword which is used by default in Yaobi.
      # 	  Therefore, we only define YAOBI_USE_FCOMI in ext/yaobi/yaobi_settings.h for 32 bit architectures.
      IF(AMD64)
        LIST(APPEND RW_CXX_FLAGS_TMP "-DMSVC_AMD64")
      ENDIF()
    ENDIF ()
	
	# Set necessary options for Win32 environments if static version of Xerces is used
	IF(XERCES_USE_STATIC_LIBS)
		LIST(APPEND RW_CXX_FLAGS_TMP "-DXERCES_STATIC_LIBRARY")
	ENDIF()
	
	SET(RW_CXX_FLAGS "${RW_CXX_FLAGS_TMP}"
		CACHE STRING "Change this to force using your own 
					  flags and not those of RobWork"
	)
	IF(DEFINED RW_CXX_FLAGS_EXTRA)
	  LIST(APPEND RW_CXX_FLAGS ${RW_CXX_FLAGS_EXTRA})
	ENDIF()
ENDIF()
ADD_DEFINITIONS(${RW_CXX_FLAGS})
MESSAGE(STATUS "RobWork: RW CXX flags: ${RW_CXX_FLAGS}")

#
# Set extra linker flags. The user should be able to change this
#
IF(NOT DEFINED RW_LINKER_FLAGS)
	# Set necessary linker options for Win32 environments if static version of Xerces is used
	IF(MSVC AND XERCES_USE_STATIC_LIBS)
	    IF(NOT IS_RELEASE)
          SET(RW_LINKER_FLAGS "/NODEFAULTLIB:LIBCMTD")
        ELSE()
          SET(RW_LINKER_FLAGS "/NODEFAULTLIB:LIBCMT")
        ENDIF()
	ENDIF()
ENDIF()
SET(RW_BUILD_WITH_LINKER_FLAGS "${RW_LINKER_FLAGS}") 
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
IF(WIN32)
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${RW_LINKER_FLAGS}" CACHE STRING "" FORCE)
ENDIF()
MESSAGE(STATUS "RobWork: RW linker flags: ${RW_LINKER_FLAGS}")

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
    ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY} 
    ${RW_CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
    ${Boost_LIBRARY_DIRS}
    ${XERCESC_LIBRARY_DIRS}
    ${YAOBI_LIBRARY_DIRS}
    ${PQP_LIBRARY_DIRS}
    ${LUA_LIBRARY_DIRS}
    ${TOLUA_LIBRARY_DIRS}
    ${LAPACK_BLAS_LIBRARY_DIRS}
)



#
# Setup the Library List here. We need to make sure the correct order is maintained
# which is crucial for some compilers.
# 
SET(ROBWORK_LIBRARIES_TMP
  ${SANDBOX_LIB}
  rw_algorithms
  rw_pathplanners
  rw_pathoptimization
  rw_simulation
  rw_opengl
  ${RW_LUA_LIBS}
  ${LUA_LIBRARIES}
  rw_proximitystrategies
  ${YAOBI_LIBRARIES}
  ${PQP_LIBRARIES}  
  rw
  ${OPENGL_LIBRARIES}
  ${XERCESC_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LAPACK_LIBRARIES} 
  ${BLAS_LIBRARIES}
  rw_qhull
)

SET(ROBWORK_LIBRARIES)
FOREACH(l ${ROBWORK_LIBRARIES_TMP})
  UNSET(tmp CACHE)
  FIND_LIBRARY(tmp ${l} PATHS ${ROBWORK_LIBRARY_DIRS} NO_DEFAULT_PATH)
  IF(tmp)
    LIST(APPEND ROBWORK_LIBRARIES ${tmp})
  ELSE()
    LIST(APPEND ROBWORK_LIBRARIES ${l})
  ENDIF()
ENDFOREACH(l)