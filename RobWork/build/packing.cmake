
############################################################
##                                                         #
## Setup PACKAGING                                         #
##                                                         #
############################################################

OPTION(CPACK_PACKAGES "Set to ON to build the packages. Requires cmake >2.4" ON)
IF(CPACK_PACKAGES)

INCLUDE(InstallRequiredSystemLibraries)

# Custom settings

# Install path
IF(UNIX)	
	SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
ENDIF()
# CPU architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
	SET(ARCH "x86")
ELSE()
	SET(ARCH "amd64")
ENDIF()
# Suffix
IF(UNIX)
	SET(SUFFIX "${ARCH}")
ELSEIF(MINGW)
	SET(SUFFIX "MinGW-${ARCH}")
ELSEIF(MSVC)
	IF(MSVC80)
		SET(SUFFIX "MSVC2005-${ARCH}")
	ELSEIF(MSVC90)
		SET(SUFFIX "MSVC2008-${ARCH}")
	ELSEIF(MSVC10)
		SET(SUFFIX "MSVC2010-${ARCH}")
	ENDIF()
ELSE()
	# Trouble
	SET(SUFFIX "")
ENDIF()
SET(PROJECT_NAME "RobWork")
SET(IGNORE_PQP "/PQP/")
SET(RW_ICON "${RW_ROOT}\\\\build\\\\rw_logo_48x48.ico")
SET(RW_IMAGE "${RW_ROOT}\\\\build\\\\rw_logo_128x64.bmp")


# The CPack settings, one by one (see http://www.cmake.org/Wiki/CMake:CPackConfiguration)

# Basic settings
IF(UNIX)
	SET(CPACK_GENERATOR "DEB")
ELSE()
	SET(CPACK_GENERATOR "NSIS")
ENDIF()
SET(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
#SET(CPACK_INSTALL_CMAKE_PROJECTS "")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RW_ROOT}/ReadMe.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWork")
SET(CPACK_PACKAGE_EXECUTABLES "" "")
IF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        SET(CPACK_PACKAGE_FILE_NAME "RobWork-${ROBWORK_VERSION}-${SUFFIX}")
ELSE("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        SET(CPACK_PACKAGE_FILE_NAME "RobWork-${ROBWORK_VERSION}-${SUFFIX}-Debug")
ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
IF(NOT UNIX)
	SET(CPACK_PACKAGE_INSTALL_DIRECTORY "RobWork\\\\RobWork ${ROBWORK_VERSION}")
ENDIF(NOT UNIX)
#SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "")
#SET(CPACK_PACKAGE_NAME "") # ${PROJECT_NAME}
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORK_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORK_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORK_VERSION_PATCH})
#SET(CPACK_PROJECT_CONFIG_FILE "")
#SET(CPACK_SOURCE_GENERATOR "")
SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;/build/(.)+/;~"
			      "/[^tolua](/src/)*/bin/;/libs/"
			      "/CMakeFiles/"
			      "CMakeCache.txt"
			      "Makefile"
			      "cmake_install.cmake"
			      "config.cmake$;/.cproject$;/.project$;/.settings"
			      "/Release;/release"
			      "/Debug;/debug"
)
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-src")
IF(UNIX)
	SET(CPACK_SOURCE_STRIP_FILES "")
ELSE(UNIX)
	SET(CPACK_SOURCE_STRIP_FILES TRUE)
ENDIF(UNIX)
SET(CPACK_STRIP_FILES TRUE)
#SET(CPACK_SYSTEM_NAME "") # ${CMAKE_SYSTEM_NAME}


# Undocumented settings
SET(CPACK_PACKAGE_CONTACT "Jimmy Alison Jørgensen (jimali@mmmi.sdu.dk)")
SET(CPACK_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;/build/")


# Advanced settings
#SET(CPACK_CMAKE_GENERATOR "") # ${CMAKE_GENERATOR}
SET(CPACK_RESOURCE_FILE_LICENSE "${RW_ROOT}/LICENSE.txt")
SET(CPACK_RESOURCE_FILE_README "${RW_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "${RW_ROOT}/ReadMe.txt")
#SET(CPACK_PACKAGE_VERSION "")
#SET(CPACK_TOPLEVEL_TAG "")
#SET(CPACK_INSTALL_COMMANDS "")
#SET(CPACK_INSTALL_DIRECTORIES "")


# Debian settings
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
SET(CPACK_PACKAGE_DESCRIPTION "... Yet Another Robotics Library")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev (>= 1.40),
                                  libboost-date-time-dev (>= 1.40),
                                  libboost-filesystem-dev (>= 1.40),
                                  libboost-regex-dev (>= 1.40),
                                  libboost-serialization-dev (>= 1.40),
                                  libboost-system-dev (>= 1.40),
                                  libboost-thread-dev (>= 1.40),
                                  libxerces-c-dev (>= 2.8),
                                  libblas-dev (>= 1.2),
                                  liblapack-dev (>= 3.2.1),
                                  freeglut3-dev (>= 2.4.0)"
)
#SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
#SET(CPACK_DEBIAN_PACKAGE_VERSION "")


# NSIS settings
SET(CPACK_NSIS_MUI_ICON "${RW_ICON}")
SET(CPACK_NSIS_MUI_UNIICON "${RW_ICON}")
SET(CPACK_PACKAGE_ICON "${RW_IMAGE}")
SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "WriteRegExpandStr HKCU Environment RW_ROOT $INSTDIR")
SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "DeleteRegValue HKCU Environment RW_ROOT")
#SET(CPACK_NSIS_COMPRESSOR "")
SET(CPACK_NSIS_MODIFY_PATH ON)
SET(CPACK_NSIS_DISPLAY_NAME "RobWork ${ROBWORK_VERSION}")
#SET(CPACK_NSIS_INSTALLED_ICON_NAME "")
SET(CPACK_NSIS_HELP_LINK "http://groups.google.com/group/robwork")
SET(CPACK_NSIS_URL_INFO_ABOUT "http://www.robwork.org")
SET(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
#SET(CPACK_NSIS_CREATE_ICONS_EXTRA "")
#SET(CPACK_NSIS_DELETE_ICONS_EXTRA "")
IF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")
	SET(CPACK_NSIS_MENU_LINKS "apidocs/html/index.html" "API documentation")
ENDIF(IS_DIRECTORY "${RW_ROOT}/apidocs/html")

INCLUDE(CPack)

ENDIF(CPACK_PACKAGES)

