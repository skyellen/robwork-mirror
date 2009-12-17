
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
SET(CPACK_GENERATOR "ZIP")
SET(CPACK_SOURCE_GENERATOR "ZIP")

# The plain 'package' target works correctly.
SET(CPACK_IGNORE_FILES "/CVS/;/.svn/;.swp$;.#;/#;/build/")
# Since the 'package_source' target does a bold copy, define a list of
# files which should be excluded. Note that 'subpattern' matching is used,
# thus to exclude a directory use /mydir/
SET(CPACK_SOURCE_IGNORE_FILES 
    "/CVS/;/.svn/;.swp$;.#;/#;/build/;~"
    "/bin/"
    "/bin/RobWorkStudio.ini_template" EXCLUDE
    "/build/*.*" EXCLUDE
)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWorkStudio")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RWSTUDIO_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_README "${RWSTUDIO_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${RWSTUDIO_ROOT}/LICENSE.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORKSTUDIO_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORKSTUDIO_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORKSTUDIO_VERSION_PATCH})
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMAKE ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}")
SET(CPACK_STRIP_FILES TRUE)
SET(CPACK_PACKAGE_EXECUTABLES "RobWorkExec" "RobWork Executable")
INCLUDE(CPack)
ENDIF (CPACK_PACKAGES)

