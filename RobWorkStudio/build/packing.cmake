
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
# Compiler dependent suffix
IF(UNIX)
    EXECUTE_PROCESS(COMMAND uname -n OUTPUT_VARIABLE SUFFIX)
    STRING(LENGTH "${SUFFIX}" SUFFIX_LEN)
    MATH(EXPR SUFFIX_LEN "${SUFFIX_LEN} - 1")
    STRING(SUBSTRING "${SUFFIX}" 0 ${SUFFIX_LEN} SUFFIX)
    STRING(TOLOWER "${SUFFIX}" SUFFIX)
ELSEIF(MINGW)
	SET(SUFFIX "windows-mingw-${ARCH}")
ELSEIF(MSVC)
	IF(MSVC80)
		SET(SUFFIX "windows-msvc2005-${ARCH}")
	ELSEIF(MSVC90)
		SET(SUFFIX "windows-msvc2008-${ARCH}")
	ELSEIF(MSVC10)
		SET(SUFFIX "windows-msvc2010-${ARCH}")
	ENDIF()
ELSE()
	# Trouble
	MESSAGE(FATAL_ERROR "Compiler not supported for package generator!")
ENDIF()
# SVN info suffix
#SET(SUFFIX "${SUFFIX}-r${RobWorkStudio_WC_REVISION}")

SET(PROJECT_NAME "RobWorkStudio")
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
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RWS_ROOT}/ReadMe.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWorkStudio")
SET(CPACK_PACKAGE_EXECUTABLES "RobWorkStudio" "RobWorkStudio")
IF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        SET(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${ROBWORKSTUDIO_VERSION}-${SUFFIX}")
ELSE("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        SET(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${ROBWORKSTUDIO_VERSION}-${SUFFIX}-Debug")
ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
#IF(NOT UNIX)
#	SET(CPACK_PACKAGE_INSTALL_DIRECTORY "RobWork\\\\RobWorkStudio ${ROBWORKSTUDIO_VERSION}")
#ENDIF(NOT UNIX)
#SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "")
SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORKSTUDIO_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORKSTUDIO_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORKSTUDIO_VERSION_PATCH})
#SET(CPACK_PROJECT_CONFIG_FILE "")
SET(CPACK_SOURCE_GENERATOR "ZIP")
SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;/build/(.)+/;~"
			      "/[^tolua](/src/)*/bin/;/libs/"
			      "config.cmake$;/.cproject$;/.project$;/.settings"
			      "RobWorkStudio.ini$"
			      "/Release;/release"
			      "/Debug;/debug"
			      "/CMakeFiles/"
			      "CMakeCache.txt"
			      "Makefile"
			      "cmake_install.cmake"
			      "/Release;/release"
			      "/Debug;/debug"
)
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}-}-src")
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
SET(CPACK_RESOURCE_FILE_LICENSE "${RWS_ROOT}/LICENSE.txt")
SET(CPACK_RESOURCE_FILE_README "${RWS_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "${RWS_ROOT}/ReadMe.txt")
#SET(CPACK_PACKAGE_VERSION "")
#SET(CPACK_TOPLEVEL_TAG "")
#SET(CPACK_INSTALL_COMMANDS "")
#SET(CPACK_INSTALL_DIRECTORIES "")


# Debian settings
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
SET(CPACK_PACKAGE_DESCRIPTION "... Yet Another Robotics Library")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "robwork (= ${ROBWORKSTUDIO_VERSION}),
				                  libqt4-dev (>= 4.5),
				                  qt4-dev-tools (>= 4.5),
				                  libboost-program-options-dev (>= 1.40)"
)
#SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
#SET(CPACK_DEBIAN_PACKAGE_VERSION "")


# NSIS settings
SET(CPACK_NSIS_MUI_ICON "${RW_ICON}")
SET(CPACK_NSIS_MUI_UNIICON "${RW_ICON}")
SET(CPACK_PACKAGE_ICON "${RW_IMAGE}")
SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "WriteRegExpandStr HKCU Environment RWS_ROOT $INSTDIR")
SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "DeleteRegValue HKCU Environment RWS_ROOT")
#SET(CPACK_NSIS_COMPRESSOR "")
SET(CPACK_NSIS_MODIFY_PATH ON)
SET(CPACK_NSIS_DISPLAY_NAME "RobWorkStudio ${ROBWORKSTUDIO_VERSION}")
#SET(CPACK_NSIS_INSTALLED_ICON_NAME "")
SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\groups.google.com\\\\group\\\\robwork")
SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.robwork.org")
SET(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
#SET(CPACK_NSIS_CREATE_ICONS_EXTRA "")
#SET(CPACK_NSIS_DELETE_ICONS_EXTRA "")
# TODO
IF(IS_DIRECTORY "${RWS_ROOT}/apidocs/html")
	SET(CPACK_NSIS_MENU_LINKS "apidocs/html/index.html" "API documentation")
ENDIF(IS_DIRECTORY "${RWS_ROOT}/apidocs/html")

INCLUDE(CPack)

ENDIF(CPACK_PACKAGES)

