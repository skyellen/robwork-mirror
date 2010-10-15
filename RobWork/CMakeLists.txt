# Test CMake version
cmake_minimum_required(VERSION 2.6)

IF( CMAKE_VERSION VERSION_LESS 2.6.4 ) 

ELSE( CMAKE_VERSION VERSION_LESS 2.6.4)
    CMAKE_POLICY(SET CMP0011 NEW)
ENDIF( CMAKE_VERSION VERSION_LESS 2.6.4)

# The name of the project.
PROJECT(RobWork)
SET(ROBWORK_VERSION 0.5.0)
STRING( REGEX MATCHALL "[0-9]+" ROBWORK_VERSIONS ${ROBWORK_VERSION} )
LIST( GET ROBWORK_VERSIONS 0 ROBWORK_VERSION_MAJOR)
LIST( GET ROBWORK_VERSIONS 1 ROBWORK_VERSION_MINOR)
LIST( GET ROBWORK_VERSIONS 2 ROBWORK_VERSION_PATCH)

MESSAGE(STATUS "RobWork version ${ROBWORK_VERSION}")

IF (NOT RW_ROOT)
    SET(RW_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF ()

SET(ROOT ${RW_ROOT})
# setup a standard project 
INCLUDE(${RW_ROOT}/build/ProjectSetup.cmake)

# setup the install dirs TODO should be cached to allow the use to change it
IF(UNIX)
	SET(RW_INSTALL_DIR "RobWork/")
ELSE(UNIX)
	SET(RW_INSTALL_DIR "./")
ENDIF(UNIX)
SET(RW_BIN_INSTALL_DIR "${RW_INSTALL_DIR}/bin/")
SET(RW_LIB_INSTALL_DIR "${RW_INSTALL_DIR}/libs/")
SET(RW_EXT_INSTALL_DIR "${RW_INSTALL_DIR}/ext/")
SET(RW_HPP_INSTALL_DIR "${RW_INSTALL_DIR}/src/")
SET(RW_RUNTIME runtime)
SET(RW_DEVEL development)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkConfig.cmake.in
  "${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake"
)

# This sets up ROBWORK_INCLUDE_DIR and ROBWORK_LIBRARIES
INCLUDE(${RW_ROOT}/build/RobWorkSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
#MESSAGE( "${ROBWORK_INCLUDE_DIR}" )

# Subdirectories to process:
ADD_SUBDIRECTORY(ext)
ADD_SUBDIRECTORY(src)

OPTION(RW_BUILD_TESTS "Set when you want to build the tests" ${RW_BUILD_TESTS})
IF( RW_BUILD_TESTS )
    MESSAGE(STATUS "RobWork tests ENABLED!")
    INCLUDE(CTest)
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWork tests DISABLED!")
ENDIF()

# Try to find the current revision 
FIND_PACKAGE(Subversion)
IF(Subversion_FOUND)
    #Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE("Current revision is ${RobWork_WC_REVISION}")
    #Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork) # not foundd in linux
    #MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure build/RobWorkBuildConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)

# Packaging
INCLUDE(build/packing.cmake)

# install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RW_INSTALL_DIR}/")
INSTALL(FILES 
    build/FindBLASLAPACK.cmake
    build/FindPQP.cmake
    build/FindRobWork.cmake
    build/RobWorkSetup.cmake
    build/FindTolua++.cmake
    build/FindXercesC.cmake
    build/FindYaobi.cmake 
    DESTINATION "${RW_INSTALL_DIR}/build/")
INSTALL(FILES "build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake"
        DESTINATION "${RW_INSTALL_DIR}/build/")
CONFIGURE_FILE("build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake"
			   "${RW_INSTALL_DIR}/build/RobWorkBuildConfig.cmake")
INSTALL(FILES ${RW_ROOT}/src/RobWorkConfig.hpp DESTINATION ${RW_HPP_INSTALL_DIR})

# Install external libraries 
INSTALL(FILES ${Boost_LIBRARIES} DESTINATION ${RW_LIB_INSTALL_DIR})
INSTALL(FILES ${XERCESC_LIBRARIES} DESTINATION ${RW_LIB_INSTALL_DIR})

# And if possible we would also like to install all the dll's and stuff that we use
# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rw_executable_name loader-test LOCATION)
CONFIGURE_FILE(
   "${RW_ROOT}/build/dependencies.cmake.in"
   "${RW_ROOT}/build/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RW_ROOT}/build/dependencies.cmake")

# Install the example directories
INSTALL(DIRECTORY "${RW_ROOT}/example/" DESTINATION "${RW_INSTALL_DIR}/example/"
	PATTERN ".svn" EXCLUDE
	PATTERN "example/CMakeLists.txt" EXCLUDE
)

# Install the API doc if doxygen has been run
IF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")
	INSTALL(DIRECTORY "${RW_ROOT}/apidocs/html" DESTINATION "${RW_INSTALL_DIR}/apidocs"
		    PATTERN ".svn" EXCLUDE
	)
ENDIF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")