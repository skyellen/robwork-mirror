# Test CMake version
cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW) 

# The name of the project.
project(RobWorkStudio)
SET(ROBWORKSTUDIO_VERSION 0.5.1)
STRING( REGEX MATCHALL "[0-9]+" ROBWORKSTUDIO_VERSIONS ${ROBWORKSTUDIO_VERSION} )
LIST( GET ROBWORKSTUDIO_VERSIONS 0 ROBWORKSTUDIO_VERSION_MAJOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 1 ROBWORKSTUDIO_VERSION_MINOR)
LIST( GET ROBWORKSTUDIO_VERSIONS 2 ROBWORKSTUDIO_VERSION_PATCH)

MESSAGE(STATUS "RobWorkStudio version ${ROBWORKSTUDIO_VERSION}")

# We want to install in same directory as RobWork
#STRING( REGEX REPLACE "[/][^/]*([/]$|$)" "/RobWomessage("Install prefix: ${CMAKE_INSTALL_PREFIX}")rk" output ${CMAKE_INSTALL_PREFIX})
#SET(CMAKE_INSTALL_PREFIX ${output})

# Used to resolve absolute path names
IF (NOT RWS_ROOT)
    SET(RWS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    SET(RWSTUDIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
ELSE ()
    SET(RWSTUDIO_ROOT ${RWS_ROOT})
ENDIF ()

# Get the compiler architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(AMD64 1)
ELSE()
	SET(AMD64 0)
ENDIF()

#Include default settings for constructing a robwork dependent project
SET(ROOT ${RWS_ROOT})
INCLUDE(${RWS_ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it
IF(UNIX)
	SET(RWS_INSTALL_DIR "RobWorkStudio/")
ELSE(UNIX)
	SET(RWS_INSTALL_DIR "./")
ENDIF(UNIX)
SET(RWS_BIN_INSTALL_DIR "${RWS_INSTALL_DIR}/bin/")
SET(RWS_LIB_INSTALL_DIR "${RWS_INSTALL_DIR}/libs/")
SET(RWS_EXT_INSTALL_DIR "${RWS_INSTALL_DIR}/ext/")
SET(RWS_HPP_INSTALL_DIR "${RWS_INSTALL_DIR}/src/")
SET(RWS_RUNTIME runtime)
SET(RWS_DEVEL development)

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWS_ROOT}/build/RobWorkStudioSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# linking options
SET(RW_STATIC_LINK_PLUGINS ${RWS_USE_STATIC_LINK_PLUGINS})
# linking options
OPTION(RWS_USE_STATIC_LINK_COMPONENTS "Set to false if dynamic link libraries of components is needed" ON)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RWS_ROOT}/src/RobWorkStudioConfig.hpp.in
  ${RWS_ROOT}/src/RobWorkStudioConfig.hpp)

# sub directories to process
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

OPTION(RWS_BUILD_TESTS "Set when you want to build the tests" ${RWS_BUILD_TESTS})
IF( RWS_BUILD_TESTS )
    MESSAGE(STATUS "RobWorkStudio tests ENABLED!")
    INCLUDE(CTest)
    #ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWorkStudio tests DISABLED!")
ENDIF()

# Try to find the current revision 
FIND_PACKAGE(Subversion QUIET)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${RWS_ROOT} RobWorkStudio)
    MESSAGE(STATUS "RobWorkStudio: Current revision is ${RobWorkStudio_WC_REVISION}")
    #Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork) # not foundd in linux
    #MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robwork
SET(INSTALLATION FALSE)
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioConfig${CMAKE_BUILD_TYPE}.cmake"
)

SET(QTDIR_TMP "${QTDIR}")
SET(QT_HEADERS_DIR_TMP "${QT_HEADERS_DIR}")
SET(QT_INCLUDE_DIR_TMP "${QT_INCLUDE_DIR}")
SET(QT_LIBRARY_DIR_TMP "${QT_LIBRARY_DIR}")
SET(QT_BINARY_DIR_TMP "${QT_BINARY_DIR}")
IF(WIN32)
	SET(QTDIR "\${RWS_ROOT}/${RWS_BIN_INSTALL_DIR}")
	SET(QT_HEADERS_DIR "\${RWS_ROOT}/${RWS_EXT_INSTALL_DIR}/qt/include")
	SET(QT_INCLUDE_DIR "\${RWS_ROOT}/${RWS_EXT_INSTALL_DIR}/qt/include")
	SET(QT_LIBRARY_DIR "\${RWS_ROOT}/${RWS_LIB_INSTALL_DIR}")
	SET(QT_BINARY_DIR "\${RWS_ROOT}/${RWS_BIN_INSTALL_DIR}")
ENDIF()

# this is needed in the BuildConfig file
FILE(TO_CMAKE_PATH $ENV{RWS_ROOT} RWS_ROOT_ENV)

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robworkstudio
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)
# Configure build script for installer
SET(INSTALLATION TRUE)
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake"
  @ONLY
)
SET(QT_HEADERS_DIR "${QT_HEADERS_DIR_TMP}")
SET(QT_HEADERS_DIR "${QT_INCLUDE_DIR_TMP}")
SET(QT_LIBRARY_DIR "${QT_LIBRARY_DIR_TMP}")
SET(QT_BINARY_DIR "${QT_BINARY_DIR_TMP}")

# Install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RWS_INSTALL_DIR}/")
INSTALL(FILES
	build/FindRobWorkStudio.cmake
	build/RobWorkStudioSetup.cmake
	DESTINATION "${RWS_INSTALL_DIR}/build/")
INSTALL(FILES "build/RobWorkStudioBuildConfig.cmake"
		DESTINATION "${RWS_INSTALL_DIR}/build/")
INSTALL(FILES ${RWS_ROOT}/src/RobWorkStudioConfig.hpp DESTINATION ${RWS_HPP_INSTALL_DIR})
INSTALL(FILES ${RWS_ROOT}/src/RobWorkStudioConfig.hpp
			  ${RWS_ROOT}/src/RobWorkStudio.hpp
			  ${RWS_ROOT}/src/RobWorkStudioPlugin.hpp
		DESTINATION ${RWS_HPP_INSTALL_DIR})


# Install dependencies on Windows
IF(WIN32)
	# Trim the Qt library lists and take only the libraries corresponding to the build type  
  SET(QT_LIBRARIES_INSTALL "")
  SET(copy_lib TRUE)
  FOREACH(s ${QT_LIBRARIES})
    IF("${s}" STREQUAL "optimized")
      IF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        SET(copy_lib TRUE)
      ELSE()
        SET(copy_lib FALSE)
      ENDIF()
    ELSEIF("${s}" STREQUAL "debug")
      IF("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
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
	INSTALL(FILES ${QT_LIBRARIES_INSTALL} DESTINATION ${RWS_LIB_INSTALL_DIR})
	INSTALL(DIRECTORY "${QT_INCLUDE_DIR_TMP}/"
			DESTINATION "${RWS_EXT_INSTALL_DIR}/qt/include/")
	INSTALL(DIRECTORY "${QTDIR_TMP}/src/"
			DESTINATION "${RWS_EXT_INSTALL_DIR}/qt/src/"
			FILES_MATCHING PATTERN "*.h*")
ENDIF()
UNSET(QTDIR)

# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION)
CONFIGURE_FILE(
   "${RWS_ROOT}/build/dependencies.cmake.in"
   "${RWS_ROOT}/build/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RWS_ROOT}/build/dependencies.cmake")

# Install the example directories
INSTALL(DIRECTORY "${RWS_ROOT}/example/" DESTINATION "${RWS_INSTALL_DIR}/example/"
	PATTERN ".svn" EXCLUDE
	PATTERN "example/CMakeLists.txt" EXCLUDE
)

# Install the API doc if doxygen has been run
#IF(IS_DIRECTORY "${RWS_ROOT}/apidocs/html")
#	INSTALL(DIRECTORY "${RWS_ROOT}/apidocs/html" DESTINATION "${RWS_INSTALL_DIR}/apidocs/"
#			PATTERN ".svn" EXCLUDE
#	)
#ENDIF(IS_DIRECTORY "${RWS_ROOT}/apidocs/html")

# Documentation build target
IF(DEFINED USE_DOC AND USE_DOC)
  FIND_PROGRAM(doc_executable doxygen)
  IF(doc_executable)
    ADD_CUSTOM_TARGET(apidoc_rw doxygen WORKING_DIRECTORY ${RW_ROOT} )
    ADD_CUSTOM_TARGET(apidoc_rws doxygen WORKING_DIRECTORY ${RWS_ROOT} ) 
  ENDIF()

  IF( EXISTS "${RW_ROOT}/apidocs/html/index.qhp" )
    ADD_CUSTOM_TARGET(qtrwdoc qhelpgenerator ${RW_ROOT}/apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robwork-v${ROBWORK_VERSION}.qch)
  ELSEIF(doc_executable)  
    ADD_CUSTOM_TARGET(qtrwdoc qhelpgenerator ${RW_ROOT}/apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robwork-v${ROBWORK_VERSION}.qch)
    ADD_DEPENDENCIES(qtrwdoc apidoc_rw)
  ENDIF()

  IF( EXISTS "${RWS_ROOT}/apidocs/html/index.qhp" )
    ADD_CUSTOM_TARGET(qtrwsdoc qhelpgenerator ${RWS_ROOT}/apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robworkstudio-v${ROBWORK_VERSION}.qch)
  ELSEIF(doc_executable)  
    ADD_CUSTOM_TARGET(qtrwsdoc qhelpgenerator ${RWS_ROOT}/apidocs/html/index.qhp -o ${RWS_ROOT}/docs/robworkstudio-v${ROBWORK_VERSION}.qch)
    ADD_DEPENDENCIES(qtrwsdoc apidoc_rws)
  ENDIF()

  IF( TARGET qtrwdoc AND TARGET qtrwsdoc )
    CONFIGURE_FILE(${RWS_ROOT}/docs/robwork_help.qhcp.in ${RWS_ROOT}/docs/robwork_help-v${ROBWORK_VERSION}.qhcp)
    ADD_CUSTOM_TARGET(robwork_help qcollectiongenerator ${RWS_ROOT}/docs/robwork_help-v${ROBWORK_VERSION}.qhcp -o ${RWS_ROOT}/docs/robwork_help-v${ROBWORK_VERSION}.qhc )
    ADD_DEPENDENCIES(robwork_help qtrwdoc qtrwsdoc) 
  ENDIF()
ENDIF()

# Packaging
INCLUDE(build/packing.cmake)

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
