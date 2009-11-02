
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
SET(CPACK_IGNORE_FILES        "/CVS/;/.svn/;.swp$;.#;/#;/build/")

# Since the 'package_source' target does a bold copy, define a list of
# files which should be excluded. Note that 'subpattern' matching is used,
# thus to exclude a directory use /mydir/

SET(IGNORE_PQP "/PQP/")

#if we are not making a public 
SET(CPACK_SOURCE_IGNORE_FILES 
    "/CVS/;/.svn/;.swp$;.#;/#;/build/(.)+/;~"
    "/[^tolua](/src/)*/bin/;/libs/"
    "/apidocs/html/"
    "config.cmake$;/.cproject$;/.project$"
)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWork")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RW_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_README "${RW_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${RW_ROOT}/LICENSE.txt")
SET(CPACK_RESOURCE_FILE_NOTICE "${RW_ROOT}/NOTICE.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORK_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORK_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORK_VERSION_PATCH})
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMAKE ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}")
SET(CPACK_STRIP_FILES TRUE)
SET(CPACK_PACKAGE_EXECUTABLES "RobWorkExec" "RobWork Executable")
INCLUDE(CPack)
ENDIF (CPACK_PACKAGES)

