# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0045)
  cmake_policy(SET CMP0045 OLD)
endif()
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

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
PROJECT(RobWork)
SET(ROBWORK_VERSION 0.7.0)
SET(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# guard against in-source builds
#if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
#  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
#endif()

SET(CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})
INCLUDE(${RW_ROOT}/cmake/RobWorkMacros.cmake)

RW_INIT_PROJECT(${RW_ROOT} RobWork RW ${ROBWORK_VERSION}) 
RW_GET_OS_INFO()
RW_SET_INSTALL_DIRS(RobWork RW)
SET(RW_EXT_INSTALL_DIR ${INCLUDE_INSTALL_DIR}/ext/)
RW_OPTIONS(RW)

# Add an "uninstall" target
configure_file("${RW_ROOT}/cmake/uninstall_target.cmake.in"
               "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${CMAKE_BINARY_DIR}/uninstall_target.cmake")


# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
INCLUDE(${RW_ROOT}/cmake/RobWorkSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Subdirectories to process:
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

RW_SYS_INFO(INFO)
MESSAGE(STATUS "RobWork: system id=${INFO} ")

# Compile the RobWork tools folder
ADD_SUBDIRECTORY(tools)

#############################################################
# setup testing
#
CONFIGURE_FILE("${RW_ROOT}/cmake/CTestCustom.cmake.in" "CTestCustom.cmake")
# configure testing configuration
CONFIGURE_FILE("${RW_ROOT}/CTestConfig.cmake.in" "${RW_ROOT}/CTestConfig.cmake")

INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RW_IS_TESTS_ENABLED "Set when you want to build the tests" 
                   ON "${RW_BUILD_TESTS}" OFF)
IF( RW_IS_TESTS_ENABLED )
    MESSAGE(STATUS "RobWork tests ENABLED!")
    # TODO Specify SET(SITE "myname") to configure the site name to use when uploading 
    SET(BUILDNAME "${INFO}")
    ENABLE_TESTING() # Should be unnecessary due to the CMake module CTest also calls this command
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
    IF(RW_HAVE_GTEST)
    	ADD_SUBDIRECTORY(gtest)
    ENDIF()
ELSE ()
    MESSAGE(STATUS "RobWork tests DISABLED!")
ENDIF()

#############################################################
# Test if we should compile the EXAMPLES
# 
CMAKE_DEPENDENT_OPTION(RW_IS_EXAMPLES_ENABLED "Set when you want to build the examples" ON "${RW_BUILD_EXAMPLES}" OFF)
IF( RW_IS_EXAMPLES_ENABLED )
    MESSAGE(STATUS "RobWork examples ENABLED!")
    ADD_SUBDIRECTORY(example)
ELSE ()
    MESSAGE(STATUS "RobWork example DISABLED!")
ENDIF()

# Try to find the current revision 
FIND_PACKAGE(Subversion)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE(STATUS "RobWork: Current revision is ${RobWork_WC_REVISION}")
    SET(ROBWORK_REVISION ${RobWork_WC_REVISION})
ENDIF(Subversion_FOUND)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure cmake/RobWorkBuildConfig.cmake.in such that other projects might use RobWork
CONFIGURE_FILE(
  ${RW_ROOT}/cmake/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake"
  @ONLY
)

# configure version information
CONFIGURE_FILE(
  ${RW_ROOT}/cmake/RobWorkConfigVersion.cmake.in
  "${RW_ROOT}/cmake/RobWorkConfigVersion.cmake"
  @ONLY
)

CONFIGURE_FILE(
  ${RW_ROOT}/cmake/RobWorkConfig.cmake.in
  "${RW_ROOT}/cmake/RobWorkConfig.cmake"
  @ONLY
)

##################################################################
# Installation stuff
#

# configuration
INSTALL(FILES 
    ${RW_ROOT}/cmake/RobWorkConfigVersion.cmake
    ${RW_ROOT}/cmake/RobWorkConfig.cmake
    DESTINATION ${RW_INSTALL_DIR} ) 

# Install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RW_INSTALL_DIR}/")
INSTALL(FILES 
    cmake/Modules/FindBLASLAPACK.cmake
    cmake/Modules/FindPQP.cmake
    cmake/Modules/FindRobWork.cmake
    cmake/Modules/FindTolua++.cmake
    cmake/Modules/FindXercesC.cmake
    cmake/Modules/FindYaobi.cmake 
    DESTINATION ${CONFIG_INSTALL_DIR}/cmake/Modules)
   
INSTALL(FILES 
        "cmake/RobWorkBuildConfig_${RW_BUILD_TYPE}.cmake"
        "cmake/RobWorkMacros.cmake"
        DESTINATION ${RW_INSTALL_DIR}/cmake)
        
INSTALL(FILES "${RW_ROOT}/src/RobWorkConfig.hpp"
		DESTINATION "${INCLUDE_INSTALL_DIR}")
INSTALL(DIRECTORY "${RW_ROOT}/ext/boostbindings/"
		DESTINATION "${INCLUDE_INSTALL_DIR}/ext/boostbindings"
		PATTERN ".svn" EXCLUDE)

# Install dependencies on Windows
IF(WIN32)
	# Trim the Boost library lists and take only the libraries corresponding to the build type  
  SET(BOOST_LIBRARIES_INSTALL "")
  SET(copy_lib TRUE)
  FOREACH(s ${Boost_LIBRARIES})
    IF("${s}" STREQUAL "optimized")
      IF("${RW_BUILD_TYPE}" STREQUAL "Release")
        SET(copy_lib TRUE)
      ELSE()
        SET(copy_lib FALSE)
      ENDIF()
    ELSEIF("${s}" STREQUAL "debug")
      IF("${RW_BUILD_TYPE}" STREQUAL "Debug")
        SET(copy_lib TRUE)
      ELSE()
        SET(copy_lib FALSE)
      ENDIF()
    ELSEIF(copy_lib)
      LIST(APPEND BOOST_LIBRARIES_INSTALL "${s}")
    ELSE()
      SET(copy_lib TRUE)
    ENDIF()    
  ENDFOREACH(s)
	
	# Install external libraries
	INSTALL(FILES ${BOOST_LIBRARIES_INSTALL} DESTINATION ${LIB_INSTALL_DIR})
	INSTALL(DIRECTORY "${Boost_INCLUDE_DIR}/boost/"
			DESTINATION "${RW_INSTALL_DIR}/ext/boost/")
	INSTALL(FILES ${XERCESC_LIBRARIES} DESTINATION ${LIB_INSTALL_DIR})
	INSTALL(DIRECTORY "${XERCESC_INCLUDE_DIR}/"
			DESTINATION "${RW_INSTALL_DIR}/ext/xerces/")
ENDIF()

# And if possible we would also like to install all the dll's and stuff that we use
# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rw_executable_name loader-test LOCATION)
CONFIGURE_FILE(
   "${RW_ROOT}/cmake/dependencies.cmake.in"
   "${RW_ROOT}/cmake/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RW_ROOT}/cmake/dependencies.cmake")

# Install the example directories
INSTALL(DIRECTORY "${RW_ROOT}/example/" DESTINATION "${RW_INSTALL_DIR}/example/"
	PATTERN ".svn" EXCLUDE
	PATTERN "example/CMakeLists.txt" EXCLUDE
)

# Install the API doc if doxygen has been run
#IF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")
#	INSTALL(DIRECTORY "${RW_ROOT}/apidocs/html" DESTINATION "${RW_INSTALL_DIR}/apidocs"
#		    PATTERN ".svn" EXCLUDc
#	)
#ENDIF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")

# Documentation build target
FIND_PROGRAM(doc_executable doxygen)
IF(doc_executable)
  ADD_CUSTOM_TARGET(doc doxygen WORKING_DIRECTORY ${RW_ROOT} )
ENDIF()

# Packaging
INCLUDE(cmake/packing.cmake)

# Check for supported MSVC compilers
# TODO: relocate to test/CMakeLists.txt?
IF(RW_BUILD_TESTS)
  IF(MSVC)
    IF(${MSVC_VERSION} EQUAL 1400)
      SET(MSVC_SOLUTION_DIR ${RW_ROOT}/projects/VisualStudio2005/RobWork)
    ELSEIF(${MSVC_VERSION} EQUAL 1500)
      SET(MSVC_SOLUTION_DIR ${RW_ROOT}/projects/VisualStudio2008/RobWork)
    ELSEIF(${MSVC_VERSION} EQUAL 1600)
      SET(MSVC_SOLUTION_DIR ${RW_ROOT}/projects/VisualStudio2010/RobWork)
    ENDIF()
    SET(MSVC_SOLUTION_FILE ${MSVC_SOLUTION_DIR}/RobWork.sln) 
    
    # Set up tests for enclosed solutions
    IF(EXISTS ${MSVC_SOLUTION_FILE})
      MESSAGE(STATUS "Adding MSVC solution test...")
      # 64-bit not supported for MSVC 2005 yet
      IF(AMD64 AND NOT ${MSVC_VERSION} EQUAL 1400)
        ADD_TEST(NAME msvc-solution-test
                 CONFIGURATIONS Release
                 COMMAND devenv ${MSVC_SOLUTION_FILE} /Rebuild "Release|x64")
        SET_TESTS_PROPERTIES(msvc-solution-test PROPERTIES TIMEOUT 2000)
      ELSE()
        ADD_TEST(NAME msvc-solution-test
                 CONFIGURATIONS Release
                 COMMAND devenv ${MSVC_SOLUTION_FILE} /Rebuild "Release|Win32")
        SET_TESTS_PROPERTIES(msvc-solution-test PROPERTIES TIMEOUT 2000)
      ENDIF()
    ENDIF()
  ENDIF()
ENDIF()





string(REPLACE ";" "\n" TOFILE_DST "$ENV{RW_LIB_FILES_CPP}")
file(WRITE "source_files" "${TOFILE_DST}")
string(REPLACE ";" "\n" TOFILE_DST "${ROBWORK_INCLUDE_DIR}")
file(WRITE "include_dirs" "${TOFILE_DST}")

##SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
#SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")



