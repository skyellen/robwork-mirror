
############################################################
##                                                         #
## Setup PACKAGING                                         #
##                                                         #
############################################################

OPTION(CPACK_PACKAGES "Set to ON to build the packages. Requires cmake >2.4" ON)
IF(CPACK_PACKAGES)

INCLUDE(InstallRequiredSystemLibraries)

# Custom settings
IF(UNIX)
	EXEC_PROGRAM(/usr/bin/dpkg ARGS "--print-architecture" OUTPUT_VARIABLE ARCH)
ELSE(UNIX)
	SET(ARCH "i386")
ENDIF(UNIX)
SET(PROJECT_NAME "RobWork-${ROBWORK_VERSION}-${ARCH}")
SET(IGNORE_PQP "/PQP/")


# The CPack settings, one by one (see http://www.cmake.org/Wiki/CMake:CPackConfiguration)

# Basic settings
IF(UNIX)
	SET(CPACK_GENERATOR "DEB")
ELSE(UNIX)
	SET(CPACK_GENERATOR "NSIS")
ENDIF(UNIX)
SET(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
#SET(CPACK_INSTALL_CMAKE_PROJECTS "")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RW_ROOT}/ReadMe.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWork")
#SET(CPACK_PACKAGE_EXECUTABLES "")
SET(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}")
#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "")
#SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "")
SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORK_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORK_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORK_VERSION_PATCH})
#SET(CPACK_PROJECT_CONFIG_FILE "")
#SET(CPACK_SOURCE_GENERATOR "")
SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;/build/(.)+/;~"
			      "/[^tolua](/src/)*/bin/;/libs/"
			      "/apidocs/html/"
			      "/apidocs/(.)+/"
			      "/CMakeFiles/"
			      "CMakeCache.txt"
			      "Makefile"
			      "cmake_install.cmake"
			      "config.cmake$;/.cproject$;/.project$;/.settings"
			      "/Release;/release"
			      "/Debug;/debug"
)
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-src")
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
				  libboost-filesystem-dev (>= 1.40),
				  libboost-system-dev (>= 1.40),
				  libboost-thread-dev (>= 1.40),
				  libboost-program-options-dev (>= 1.40),
				  libboost-date-time-dev (>= 1.40),
				  libboost-regex-dev (>= 1.40),
				  libxerces-c-dev (>= 2.8),
				  libblas-dev (>= 1.2),
				  liblapack-dev (>= 3.2.1)"
)
#SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
#SET(CPACK_DEBIAN_PACKAGE_VERSION "")


# NSIS settings
#SET(CPACK_NSIS_MUI_ICON "")
#SET(CPACK_NSIS_MUI_UNIICON "")
SET(CPACK_PACKAGE_ICON "${RW_ROOT}/build/rw_logo_64x64.png")
#SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "")
#SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "")
#SET(CPACK_NSIS_COMPRESSOR "")
SET(CPACK_NSIS_MODIFY_PATH ON)
SET(CPACK_NSIS_DISPLAY_NAME "RobWork ${ROBWORK_VERSION}")
#SET(CPACK_NSIS_INSTALLED_ICON_NAME "")
SET(CPACK_NSIS_HELP_LINK "http://groups.google.com/group/robwork")
SET(CPACK_NSIS_URL_INFO_ABOUT "http://www.robwork.org")
SET(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
#SET(CPACK_NSIS_CREATE_ICONS_EXTRA "")
#SET(CPACK_NSIS_DELETE_ICONS_EXTRA "")
#SET(CPACK_NSIS_MENU_LINKS "")

INCLUDE(CPack)

ENDIF(CPACK_PACKAGES)

