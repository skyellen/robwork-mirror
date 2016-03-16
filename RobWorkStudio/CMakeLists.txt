# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF()

# The name of the project.
PROJECT(RobWorkStudio)
SET(ROBWORKSTUDIO_VERSION 0.7.0)
SET(RWS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
MESSAGE(STATUS "RobWorkStudio version ${ROBWORKSTUDIO_VERSION}")

# lets start by finding RobWork since this is the primary dependency and because we
# want to use the macros available in RobWork
# if RW_ROOT is defined then we use that, else we try finding it the traditional way
IF(RW_ROOT)
    SET(RobWork_DIR ${RW_ROOT}/cmake)
ELSE()
    # try finding it relative to this directory
    FIND_FILE(ROBWORKDEVEL_FOUND RobWorkSetup.cmake 
        ${RWS_ROOT}/../RobWork/cmake NO_DEFAULT_PATH)
    IF(ROBWORKDEVEL_FOUND)
        SET(RW_ROOT ${RWS_ROOT}/../RobWork/)
        SET(RobWork_DIR ${RW_ROOT}/cmake)
    ENDIF()
ENDIF()
FIND_PACKAGE(RobWork ${ROBWORKSTUDIO_VERSION} REQUIRED)

# include the macros from robwork
INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)
INCLUDE(${RWS_ROOT}/cmake/RobWorkStudioMacros.cmake)

RW_INIT_PROJECT(${RWS_ROOT} RobWorkStudio RWS ${ROBWORKSTUDIO_VERSION})
RW_GET_OS_INFO()
RW_SET_INSTALL_DIRS(RobWork RWS)
RW_OPTIONS(RWS)

# Add an "uninstall" target
configure_file("${RWS_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWS_ROOT}/cmake/RobWorkStudioSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
#MESSAGE(" ${ROBWORKSTUDIO_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} ")
# linking options
SET(RW_STATIC_LINK_PLUGINS ${RWS_USE_STATIC_LINK_PLUGINS})
# linking options
OPTION(RWS_USE_STATIC_LINK_COMPONENTS "Set to false if dynamic link libraries of components is needed" ON)

# sub directories to process
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

RW_SYS_INFO(INFO)
MESSAGE(STATUS "RobWorkStudio: platform ${INFO} ")

#############################################################
# setup testing
#
CONFIGURE_FILE("${RWS_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
# configure testing configuration
CONFIGURE_FILE("${RWS_ROOT}/CTestConfig.cmake.in" "${RWS_ROOT}/CTestConfig.cmake")

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RWS_IS_TESTS_ENABLED "Set when you want to build the tests" 
                   ON "${RWS_BUILD_TESTS}" OFF)
IF( RWS_IS_TESTS_ENABLED )
    MESSAGE(STATUS "RobWorkStudio: tests ENABLED!")
    INCLUDE(CTest)
    #ADD_SUBDIRECTORY(test)
    IF(RW_BUILD_WITH_GTEST)
    	ADD_SUBDIRECTORY(gtest)
    ENDIF()
ELSE ()
    MESSAGE(STATUS "RobWorkStudio: tests DISABLED!")
ENDIF()

###############################################################
# Try to find the current revision 
#
FIND_PACKAGE(Subversion QUIET)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${RWS_ROOT} RobWorkStudio)
    MESSAGE(STATUS "RobWorkStudio: Current revision is ${RobWorkStudio_WC_REVISION}")
    SET(ROBWORKSTUDIO_REVISION ${RobWorkStudio_WC_REVISION})
ELSE()
    MESSAGE(STATUS "RobWorkStudio: No revision info!")
ENDIF(Subversion_FOUND)

###############################################################
# CONFIGURATION
# configure build/RobWorkStudioConfig.cmake.in 
#

# first configure the header file
CONFIGURE_FILE(
  ${RWS_ROOT}/src/RobWorkStudioConfig.hpp.in
  "${RWS_ROOT}/src/RobWorkStudioConfig.hpp"
  @ONLY
)

# next build information script
CONFIGURE_FILE(
  ${RWS_ROOT}/cmake/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/cmake/RobWorkStudioBuildConfig_${RW_BUILD_TYPE}.cmake"
  @ONLY
)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robworkstudio
CONFIGURE_FILE(
  ${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake.in
  "${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake" @ONLY)

# and the version info
CONFIGURE_FILE(
  ${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake.in
  "${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake" @ONLY)

##################################################################
# Installation stuff
#

# install configuration 
INSTALL(FILES 
    ${RWS_ROOT}/cmake/RobWorkStudioConfigVersion.cmake
    ${RWS_ROOT}/cmake/RobWorkStudioConfig.cmake
    DESTINATION ${RWS_INSTALL_DIR} ) 

INSTALL(FILES 
        "cmake/RobWorkStudioBuildConfig_${RWS_BUILD_TYPE}.cmake"
        "cmake/RobWorkStudioMacros.cmake"
        DESTINATION ${RWS_INSTALL_DIR})

# misc
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt 
        DESTINATION "${RWS_INSTALL_DIR}/")
INSTALL(FILES ${RWS_ROOT}/src/RobWorkStudioConfig.hpp
			  ${RWS_ROOT}/src/RobWorkStudio.hpp
			  ${RWS_ROOT}/src/RobWorkStudioPlugin.hpp
		DESTINATION ${INCLUDE_INSTALL_DIR})


# Install dependencies on Windows
IF(WIN32)
	# Trim the Qt library lists and take only the libraries corresponding to the build type  
  SET(QT_LIBRARIES_INSTALL "")
  SET(copy_lib TRUE)
  RW_IS_RELEASE(IS_RELEASE)
  FOREACH(s ${QT_LIBRARIES})
    IF("${s}" STREQUAL "optimized")
      IF( ${IS_RELEASE} )
        SET(copy_lib TRUE)
      ELSE()
        SET(copy_lib FALSE)
      ENDIF()
    ELSEIF("${s}" STREQUAL "debug")
      IF(NOT ${IS_RELEASE})
        SET(copy_lib TRUE)
      ELSE()
        SET(copy_lib FALSE)
      ENDIF()
    ELSEIF(copy_lib)
      LIST(APPEND QT_LIBRARIES_INSTALL "${s}")
    ELSE()
      SET(copy_lib TRUE)
    ENDIF()    
  ENDFOREACH(s)

	# Install external libraries
	#INSTALL(FILES ${QT_LIBRARIES_INSTALL} DESTINATION ${LIB_INSTALL_DIR})
	#INSTALL(DIRECTORY "${QT_INCLUDE_DIR_TMP}/"
	#		DESTINATION "${EXT_INSTALL_DIR}/qt/include/")
	#INSTALL(DIRECTORY "${QTDIR_TMP}/src/"
	#		DESTINATION "${EXT_INSTALL_DIR}/qt/src/"
	#		FILES_MATCHING PATTERN "*.h*")
ENDIF()
UNSET(QTDIR)

# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION)
CONFIGURE_FILE(
   "${RWS_ROOT}/cmake/dependencies.cmake.in"
   "${RWS_ROOT}/cmake/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RWS_ROOT}/cmake/dependencies.cmake")

# Install the example directories
INSTALL(DIRECTORY "${RWS_ROOT}/example/" DESTINATION "${RWS_INSTALL_DIR}/example/"
	PATTERN ".svn" EXCLUDE
	PATTERN "example/CMakeLists.txt" EXCLUDE
)

# Documentation build target
FIND_PROGRAM(doc_executable doxygen)
IF(doc_executable)
  ADD_CUSTOM_TARGET(apidoc doxygen WORKING_DIRECTORY ${RW_ROOT}/.. )

  IF( EXISTS "${RW_ROOT}/../apidocs/html/index.qhp" )
    ADD_CUSTOM_TARGET(qtrwdoc qhelpgenerator ${RW_ROOT}/../apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robwork-v${ROBWORKSTUDIO_VERSION}.qch)
  ELSEIF(doc_executable)  
    ADD_CUSTOM_TARGET(qtrwdoc qhelpgenerator ${RW_ROOT}/../apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robwork-v${ROBWORKSTUDIO_VERSION}.qch)
    ADD_DEPENDENCIES(qtrwdoc apidoc_rw)
  ENDIF()
  
ENDIF()

# Packaging
INCLUDE(cmake/packing.cmake)

# Dashboard
#IF(DEFINED USE_CDASH AND USE_CDASH)
	CONFIGURE_FILE("${RWS_ROOT}/CTestConfig.cmake.in" "${RWS_ROOT}/CTestConfig.cmake")
#ENDIF()

# Check for supported MSVC compilers
# TODO: relocate to test/CMakeLists.txt?
IF(RW_BUILD_TESTS)
  IF(MSVC)
    IF(${MSVC_VERSION} EQUAL 1400)
      SET(MSVC_SOLUTION_DIR ${RWS_ROOT}/projects/VisualStudio2005/RobWorkStudio)
    ELSEIF(${MSVC_VERSION} EQUAL 1500)
      SET(MSVC_SOLUTION_DIR ${RWS_ROOT}/projects/VisualStudio2008/RobWorkStudio)
    ELSEIF(${MSVC_VERSION} EQUAL 1600)
      SET(MSVC_SOLUTION_DIR ${RWS_ROOT}/projects/VisualStudio2010/RobWorkStudio)
    ENDIF()
    SET(MSVC_SOLUTION_FILE ${MSVC_SOLUTION_DIR}/RobWorkStudio.sln)
    
    # Set up tests for enclosed solutions
    IF(EXISTS ${MSVC_SOLUTION_FILE})
      MESSAGE(STATUS "Adding MSVC solution test...")
      # 64-bit not supported for MSVC 2005 yet
      IF(AMD64 AND NOT ${MSVC_VERSION} EQUAL 1400)
        ADD_TEST(NAME msvc-solution-test
                 CONFIGURATIONS Release
                 COMMAND devenv ${MSVC_SOLUTION_FILE} /Rebuild "Release|x64")
      ELSE()
        ADD_TEST(NAME msvc-solution-test
                 CONFIGURATIONS Release
                 COMMAND devenv ${MSVC_SOLUTION_FILE} /Rebuild "Release|Win32")
      ENDIF()
    ENDIF()
  ENDIF()
ENDIF()
