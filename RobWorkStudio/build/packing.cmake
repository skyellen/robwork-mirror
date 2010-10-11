
############################################################
##                                                         #
## Setup PACKAGING                                         #
##                                                         #
############################################################

OPTION( CPACK_PACKAGES "Set to ON to build the packages. Requires cmake >2.4" ON )
IF (CPACK_PACKAGES)

INCLUDE(InstallRequiredSystemLibraries)

# ATTENTION: There is sometimes a _SOURCE_ version of an
# option as well, set both if necessary !

# Create .tar.gz and .tar.tbz2 files:
IF(UNIX)
	SET(CPACK_GENERATOR "DEB")
	SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libqt4-dev (>= 4.5),
					  qt4-dev-tools (>= 4.5),
					  libboost-dev (>= 1.40),
					  libboost-filesystem-dev (>= 1.40),
					  libboost-system-dev (>= 1.40),
					  libboost-thread-dev (>= 1.40),
					  libboost-program-options-dev (>= 1.40),
					  libboost-date-time-dev (>= 1.40),
					  libboost-regex-dev (>= 1.40),
					  libxerces-c-dev (>= 2.8),
					  libblas-dev (>= 1.2),
					  liblapack-dev (>= 3.2.1)")
	EXEC_PROGRAM(/usr/bin/dpkg ARGS "--print-architecture" OUTPUT_VARIABLE ARCH)
ELSE(UNIX)
	SET(CPACK_GENERATOR "NSIS")
	SET(ARCH "i386")
ENDIF(UNIX)

#SET(CPACK_SOURCE_GENERATOR "ZIP")

# The plain 'package' target works correctly.
SET(CPACK_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;")
# Since the 'package_source' target does a bold copy, define a list of
# files which should be excluded. Note that 'subpattern' matching is used,
# thus to exclude a directory use /mydir/
SET(CPACK_SOURCE_IGNORE_FILES 
    "/CVS/;/.svn/;.swp$;.#;/#;/build/(.)+/;~"
    "/[^tolua](/src/)*/bin/;/libs/"
    "/apidocs/html/"
    "/apidocs/(.)+/"
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

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWorkStudio")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RWSTUDIO_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${RWSTUDIO_ROOT}/LICENSE.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORKSTUDIO_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORKSTUDIO_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORKSTUDIO_VERSION_PATCH})

SET(CPACK_PACKAGE_NAME "RobWorkStudio-${ROBWORKSTUDIO_VERSION}-${ARCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-src")
SET(CPACK_PACKAGE_CONTACT "jimali@mmmi.sdu.dk")
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}")

SET(CPACK_RESOURCE_FILE_README "${RWSTUDIO_ROOT}/ReadMe.txt")
SET(CPACK_STRIP_FILES TRUE)

SET(CPACK_PACKAGE_INSTALL_DIRECTORY RobWorkStudio)
#SET(CPACK_PACKAGE_EXECUTABLES "RobWorkExec" "RobWork Executable")

INCLUDE(CPack)
ENDIF (CPACK_PACKAGES)

