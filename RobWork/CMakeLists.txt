# Test CMake version
cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW)

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
SET(RW_BIN_INSTALL_DIR "bin/${CMAKE_BUILD_TYPE}/")
SET(RW_LIB_INSTALL_DIR "libs/${CMAKE_BUILD_TYPE}/")
SET(RW_EXT_INSTALL_DIR "ext/")
SET(RW_HPP_INSTALL_DIR "src/")
SET(RW_RUNTIME runtime)
SET(RW_DEVEL development)

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkConfig.cmake.in
  "${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

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

# Configure build/RobWorkConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake")

# Packaging
INCLUDE(build/packing.cmake)

# install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "./")
INSTALL(FILES 
    build/FindBLASLAPACK.cmake
    build/FindPQP.cmake
    build/FindRobWork.cmake
    build/RobWorkSetup.cmake
    build/FindTolua++.cmake
    build/FindXercesC.cmake
    build/FindYaobi.cmake 
    DESTINATION "build/")
INSTALL(FILES "build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake" DESTINATION "build/" )
INSTALL(FILES ${RW_ROOT}/src/RobWorkConfig.hpp DESTINATION ${RW_HPP_INSTALL_DIR})

# install external libraries 
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

# allso install the example directories
INSTALL(FILES 
    "${RW_ROOT}/example/consoleapp/CMakeLists.txt"
    "${RW_ROOT}/example/consoleapp/SampleTest.cpp"
    "${RW_ROOT}/example/consoleapp/README.txt"
    DESTINATION "example/consoleapp/")

INSTALL(FILES 
    "${RW_ROOT}/example/libraryapp/CMakeLists.txt"
    "${RW_ROOT}/example/libraryapp/README.txt"
    DESTINATION "example/libraryapp/")

INSTALL(FILES 
    "${RW_ROOT}/example/libraryapp/src/SampleObj1.hpp"
    "${RW_ROOT}/example/libraryapp/src/SampleObj1.cpp"
    "${RW_ROOT}/example/libraryapp/src/SampleObj2.hpp"
    "${RW_ROOT}/example/libraryapp/src/SampleObj2.cpp"
    "${RW_ROOT}/example/libraryapp/src/CMakeLists.txt"
    DESTINATION "example/libraryapp/src/")

INSTALL(FILES 
    "${RW_ROOT}/example/simplegui/CMakeLists.txt"
    "${RW_ROOT}/example/simplegui/ViewerExample.cpp"
    "${RW_ROOT}/example/simplegui/README.txt"
    DESTINATION "example/simplegui/")
    
# Install all headerfiles from sandbox
INSTALL(DIRECTORY "${RW_ROOT}/example/simplegui/src" DESTINATION "example/simplegui/src/" 
    FILES_MATCHING 
        PATTERN "*.h" 
        PATTERN "*.hpp"
        PATTERN "*.cpp"
        PATTERN ".svn" EXCLUDE
)



