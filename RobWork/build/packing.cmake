
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
    "/apidocs/(.)+/"
    "/CMakeFiles/"
    "CMakeCache.txt"
    "Makefile"
    "cmake_install.cmake"
    "config.cmake$;/.cproject$;/.project$;/.settings"
    "/Release;/release"
    "/Debug;/debug"
)

SET(CPACK_PACKAGE_NAME "RobWork-${ROBWORK_VERSION}-${CMAKE_SYSTEM}")
SET(CPACK_PACKAGE_FILE_NAME "RobWork-${ROBWORK_VERSION}-${CMAKE_SYSTEM}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "RobWork")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RobWork")
SET(CPACK_PACKAGE_VENDOR "The RobWork Community")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${RW_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_README "${RW_ROOT}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${RW_ROOT}/LICENSE.txt")
SET(CPACK_RESOURCE_FILE_NOTICE "${RW_ROOT}/NOTICE.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR ${ROBWORK_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${ROBWORK_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${ROBWORK_VERSION_PATCH})
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "RobWork")
SET(CPACK_STRIP_FILES TRUE)

IF(WIN32 AND NOT UNIX)
  # There is a bug in NSI that does not handle full unix paths properly. Make
  # sure there is at least one set of four (4) backlasshes.
  SET(CPACK_PACKAGE_ICON "${RW_ROOT}/build/rw_logo_64x64.png")
  SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\RobWork.exe")
  SET(CPACK_NSIS_DISPLAY_NAME "RobWork ${ROBWORK_VERSION}")
  SET(CPACK_NSIS_HELP_LINK "http://groups.google.com/group/robwork")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http://www.robwork.org")
  SET(CPACK_NSIS_CONTACT "robwork@googlegroups.com")
  SET(CPACK_NSIS_MODIFY_PATH ON)
ELSE(WIN32 AND NOT UNIX)
  SET(CPACK_STRIP_FILES "bin/MyExecutable")
  SET(CPACK_SOURCE_STRIP_FILES "")
ENDIF(WIN32 AND NOT UNIX)

SET(CPACK_PACKAGE_EXECUTABLES "RobWorkExec" "RobWork Executable")
INCLUDE(CPack)
ENDIF (CPACK_PACKAGES)

