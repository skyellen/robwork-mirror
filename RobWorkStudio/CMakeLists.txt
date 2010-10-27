# Test CMake version
cmake_minimum_required(VERSION 2.6)

CMAKE_POLICY(SET CMP0011 NEW) 

# The name of the project.
project(RobWorkStudio)
SET(ROBWORKSTUDIO_VERSION 0.5.0)
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
ENDIF ()

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

# Configure build/RobWorkStudioConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioConfig${CMAKE_BUILD_TYPE}.cmake"
)

# This sets up ROBWORKSTUDIO_INCLUDE_DIR and ROBWORKSTUDIO_LIBRARIES
# RobWork dependencies are included in these vars
INCLUDE(${RWS_ROOT}/build/RobWorkStudioSetup.cmake)

INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIR} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
#MESSAGE("${ROBWORKSTUDIO_LIBRARY_DIRS}")

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
    ADD_SUBDIRECTORY(test)
ELSE ()
    MESSAGE(STATUS "RobWorkStudio tests DISABLED!")
ENDIF()


# Configure build/RobWorkStudioBuildConfig.cmake.in such that other projects might use robwork
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake"
)
CONFIGURE_FILE(
  ${RWS_ROOT}/build/RobWorkStudioBuildConfig.cmake.in
  "${RWS_ROOT}/build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)

# Documentation
#FIND_PACKAGE(Doxygen)
#IF( DOXYGEN_FOUND )
#    ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${ROOT}/Doxyfile)
#ELSE ()
#    MESSAGE(STATUS "Documentation target is not added! To enable doxygen, make sure doxygen.exe is in the path.")
#ENDIF( )

# Try to find the current revision 
FIND_PACKAGE(Subversion QUIET)
IF(Subversion_FOUND)
    Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE(STATUS "Current revision is ${RobWork_WC_REVISION}")
    #Subversion_WC_LOG(${PROJECT_SOURCE_DIR} RobWork) # not foundd in linux
    #MESSAGE("Last changed log is ${Project_LAST_CHANGED_LOG}")
ENDIF(Subversion_FOUND)

# Install stuff
INSTALL(FILES LICENSE.txt NOTICE.txt ChangeLog.txt DESTINATION "${RWS_INSTALL_DIR}/")
INSTALL(FILES build/FindRobWorkStudio.cmake DESTINATION "${RWS_INSTALL_DIR}/build/")
INSTALL(FILES "build/RobWorkStudioBuildConfig.cmake" "build/RobWorkStudioBuildConfig${CMAKE_BUILD_TYPE}.cmake"
		DESTINATION "${RWS_INSTALL_DIR}/build/")
INSTALL(FILES ${RWS_ROOT}/src/RobWorkStudioConfig.hpp DESTINATION ${RWS_HPP_INSTALL_DIR})

# Install dependencies on Windows
IF(WIN32)
	# Trim the Qt library lists and take only the libraries corresponding to the build type
	SET(labeled FALSE)
	LIST(GET QT_LIBRARIES 0 zero_elem)
	IF("${zero_elem}" STREQUAL "optimized" OR "${zero_elem}" STREQUAL "debug")
		SET(labeled TRUE)
	ENDIF()
	IF(LABELED)
		SET(QT_LIBRARIES_INSTALL "")
		SET(copy_lib FALSE)
		FOREACH(s ${QT_LIBRARIES})
				IF(NOT labeled)
					# If library list does not contain labels, always copy
					LIST(APPEND QT_LIBRARIES_INSTALL "${s}")			
				ELSE()
					# Else skip labels and copy only according to build type
					IF(copy_lib)
						LIST(APPEND QT_LIBRARIES_INSTALL "${s}")
						SET(copy_lib FALSE)
					ENDIF()
					
					# List entry matches "optimized", copy in next iteration if in release mode
					IF("${s}" STREQUAL "optimized" AND "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
						SET(copy_lib TRUE)
					# List entry matches "debug", copy in next iteration if in debug mode
					ELSEIF("${s}" STREQUAL "debug" AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
						SET(copy_lib TRUE)
					ENDIF()
				ENDIF()
		ENDFOREACH(s)
	ELSE()
		SET(QT_LIBRARIES_INSTALL "${QT_LIBRARIES}")
	ENDIF()

	# Install external libraries
	INSTALL(FILES ${QT_LIBRARIES_INSTALL} DESTINATION ${RWS_LIB_INSTALL_DIR})
ENDIF()

# this will configure the dependency file that will be used if install is invoked
GET_TARGET_PROPERTY(rwstudio_executable_name RobWorkStudio LOCATION)
CONFIGURE_FILE(
   "${RWSTUDIO_ROOT}/build/dependencies.cmake.in"
   "${RWSTUDIO_ROOT}/build/dependencies.cmake"
   @ONLY
)

INSTALL(SCRIPT "${RWSTUDIO_ROOT}/build/dependencies.cmake")

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

# Packaging
INCLUDE(build/packing.cmake)
