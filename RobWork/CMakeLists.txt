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

# Get the compiler architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(AMD64 TRUE)
ELSE()
	SET(AMD64 FALSE)
ENDIF()

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
    Subversion_WC_INFO(${RW_ROOT} RobWork)
    MESSAGE(STATUS "Current revision is ${RobWork_WC_REVISION}")
ENDIF(Subversion_FOUND)

# Store the test results in a RobWorkConfig.hpp file.
CONFIGURE_FILE(
  ${RW_ROOT}/src/RobWorkConfig.hpp.in
  ${RW_ROOT}/src/RobWorkConfig.hpp)

# Configure build/RobWorkBuildConfig.cmake.in such that other projects might use robwork
SET(INSTALLATION FALSE)
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/build/RobWorkBuildConfig${CMAKE_BUILD_TYPE}.cmake"
)

# Store the build paths in a temporary list
SET(VARS_TMP "${TOLUA_CMD}" "${TOLUA_INCLUDE_DIR}" "${LUA_INCLUDE_DIR}" "${PQP_INCLUDE_DIR}" "${YAOBI_INCLUDE_DIR}"
			 "${XERCESC_INCLUDE_DIR}" "${XERCESC_LIB_DIR}" "${BOOST_ROOT}" "${BOOST_INCLUDEDIR}" "${BOOST_LIBRARYDIR}"
	)
SET(ROBWORK_LIBRARIES_TMP "${ROBWORK_LIBRARIES}")
SET(ROBWORK_LIBRARY_DIRS_TMP "${ROBWORK_LIBRARY_DIRS}")
SET(ROBWORK_INCLUDE_DIR_TMP "${ROBWORK_INCLUDE_DIR}")
# Make all build paths relative to install directory
IF(WIN32)
	SET(TOLUA_CMD "\${RW_ROOT}/${RW_BIN_INSTALL_DIR}/tolua")
	SET(TOLUA_INCLUDE_DIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/tolua/include")
	SET(LUA_INCLUDE_DIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/lua/src")
	SET(PQP_INCLUDE_DIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/pqp")
	SET(YAOBI_INCLUDE_DIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/yaobi")
	SET(XERCESC_INCLUDE_DIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/xerces")
	SET(XERCESC_LIB_DIR "\${RW_ROOT}/${RW_LIB_INSTALL_DIR}")
	SET(BOOST_ROOT "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/boost/")
	SET(BOOST_INCLUDEDIR "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}/boost/")
	SET(BOOST_LIBRARYDIR "\${RW_ROOT}/${RW_LIB_INSTALL_DIR}")
	SET(ROBWORK_LIBRARIES "")
	FOREACH(lib ${ROBWORK_LIBRARIES_TMP})
		GET_FILENAME_COMPONENT(lib_rel "${lib}" NAME)
		LIST(APPEND ROBWORK_LIBRARIES "\${RW_ROOT}/libs/${lib_rel}")
	ENDFOREACH(lib)
	SET(ROBWORK_LIBRARY_DIRS "\${RW_ROOT}/${RW_LIB_INSTALL_DIR}")
	SET(ROBWORK_INCLUDE_DIR "${TOLUA_INCLUDE_DIR}" "${LUA_INCLUDE_DIR}"
							"${PQP_INCLUDE_DIR}" "${YAOBI_INCLUDE_DIR}"
							"${XERCESC_INCLUDE_DIR}" "${BOOST_INCLUDEDIR}"
							"\${RW_ROOT}/${RW_HPP_INSTALL_DIR}" "\${RW_ROOT}/${RW_EXT_INSTALL_DIR}"
		)
ELSEIF(UNIX)
	SET(TOLUA_CMD "${CMAKE_INSTALL_PREFIX}/${RW_BIN_INSTALL_DIR}/tolua")
	SET(TOLUA_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${RW_EXT_INSTALL_DIR}/tolua/include")
	SET(LUA_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${RW_EXT_INSTALL_DIR}/lua/src")
	SET(PQP_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${RW_EXT_INSTALL_DIR}/pqp")
	SET(YAOBI_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${RW_EXT_INSTALL_DIR}/yaobi")
	SET(ROBWORK_LIBRARIES "")
	FOREACH(lib ${ROBWORK_LIBRARIES_TMP})
		GET_FILENAME_COMPONENT(lib_rel "${lib}" NAME)
		LIST(APPEND ROBWORK_LIBRARIES "${lib_rel}")
	ENDFOREACH(lib)
	SET(ROBWORK_LIBRARY_DIRS "/usr/lib" "/usr/local/lib" "${CMAKE_INSTALL_PREFIX}/${RW_LIB_INSTALL_DIR}")
	SET(ROBWORK_INCLUDE_DIR "/usr/include" "/usr/local/include"
                            "${TOLUA_INCLUDE_DIR}" "${LUA_INCLUDE_DIR}"
							"${PQP_INCLUDE_DIR}" "${YAOBI_INCLUDE_DIR}"
							"${CMAKE_INSTALL_PREFIX}/${RW_HPP_INSTALL_DIR}" "${CMAKE_INSTALL_PREFIX}/${RW_EXT_INSTALL_DIR}"
		)
ENDIF()
# Configure build script for installer
SET(INSTALLATION TRUE)
CONFIGURE_FILE(
  ${RW_ROOT}/build/RobWorkBuildConfig.cmake.in
  "${RW_ROOT}/build/RobWorkBuildConfig.cmake"
  @ONLY
)
# Restore the build paths
LIST(GET VARS_TMP 0 TOLUA_CMD)
LIST(GET VARS_TMP 1 TOLUA_INCLUDE_DIR)
LIST(GET VARS_TMP 2 LUA_INCLUDE_DIR)
LIST(GET VARS_TMP 3 PQP_INCLUDE_DIR)
LIST(GET VARS_TMP 4 YAOBI_INCLUDE_DIR)
LIST(GET VARS_TMP 5 XERCESC_INCLUDE_DIR)
LIST(GET VARS_TMP 6 XERCESC_LIB_DIR)
LIST(GET VARS_TMP 7 BOOST_ROOT)
LIST(GET VARS_TMP 8 BOOST_INCLUDEDIR)
LIST(GET VARS_TMP 9 BOOST_LIBRARYDIR)
SET(ROBWORK_LIBRARIES "${ROBWORK_LIBRARIES_TMP}")
SET(ROBWORK_LIBRARY_DIRS "${ROBWORK_LIBRARY_DIRS_TMP}")
SET(ROBWORK_INCLUDE_DIR "${ROBWORK_INCLUDE_DIR_TMP}")

# Install stuff
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
INSTALL(FILES "build/RobWorkBuildConfig.cmake"
        DESTINATION "${RW_INSTALL_DIR}/build/")
INSTALL(FILES "${RW_ROOT}/src/RobWorkConfig.hpp"
		DESTINATION "${RW_HPP_INSTALL_DIR}")
INSTALL(DIRECTORY "${RW_ROOT}/ext/boost/"
		DESTINATION "${RW_EXT_INSTALL_DIR}/boost/"
		PATTERN ".svn" EXCLUDE)

# Install dependencies on Windows
IF(WIN32)
	# Trim the Boost library lists and take only the libraries corresponding to the build type
	SET(labeled FALSE)
	LIST(GET Boost_LIBRARIES 0 zero_elem)
	IF("${zero_elem}" STREQUAL "optimized" OR "${zero_elem}" STREQUAL "debug")
		SET(labeled TRUE)
	ENDIF()
	IF(labeled)
		SET(BOOST_LIBRARIES_INSTALL "")
		SET(copy_lib FALSE)
		FOREACH(s ${Boost_LIBRARIES})
				IF(NOT labeled)
					# If library list does not contain labels, always copy
					LIST(APPEND BOOST_LIBRARIES_INSTALL "${s}")			
				ELSE()
					# Else skip labels and copy only according to build type
					IF(copy_lib)
						LIST(APPEND BOOST_LIBRARIES_INSTALL "${s}")
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
		SET(BOOST_LIBRARIES_INSTALL "${Boost_LIBRARIES}")
	ENDIF()
	
	# Install external libraries
	INSTALL(FILES ${BOOST_LIBRARIES_INSTALL} DESTINATION ${RW_LIB_INSTALL_DIR})
	INSTALL(DIRECTORY "${Boost_INCLUDE_DIR}/boost/"
			DESTINATION "${RW_EXT_INSTALL_DIR}/boost/boost/")
	INSTALL(FILES ${XERCESC_LIBRARIES} DESTINATION ${RW_LIB_INSTALL_DIR})
	INSTALL(DIRECTORY "${XERCESC_INCLUDE_DIR}/"
			DESTINATION "${RW_EXT_INSTALL_DIR}/xerces/")
ENDIF()

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
#IF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")
#	INSTALL(DIRECTORY "${RW_ROOT}/apidocs/html" DESTINATION "${RW_INSTALL_DIR}/apidocs"
#		    PATTERN ".svn" EXCLUDE
#	)
#ENDIF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")

# Packaging
INCLUDE(build/packing.cmake)

# Dashboard
IF(DEFINED USE_CDASH)
	CONFIGURE_FILE("${RW_ROOT}/CTestConfig.cmake.in" "${RW_ROOT}/CTestConfig.cmake")
ENDIF()
