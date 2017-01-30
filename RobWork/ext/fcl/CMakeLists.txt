option(FCL_WITH_OCTOMAP "octomap library support" OFF)
option(FCL_BUILD_TESTS "Build FCL tests" OFF)

# If ccd can not be found, we compile internal ccd target.
# Find ccd
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(CCD ccd QUIET)
  # check to see if the pkg is installed under the libccd name
  if(NOT CCD_FOUND)
    pkg_check_modules(CCD libccd QUIET)
  endif()
endif()

if(NOT CCD_FOUND)
    # if pkgconfig is not installed, then fall back on more fragile detection
    # of ccd
    find_path(CCD_INCLUDE_DIRS ccd/ccd.h)
    find_library(CCD_LIBRARY ccd)
    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY)
        set(CCD_LIBRARIES "${CCD_LIBRARY}")
    else()
        # Compile CCD internal target
        MESSAGE(STATUS "Compiling internal ccd target for FCL.")
        add_subdirectory(libccd)
        SET(CCD_INCLUDE_DIRS "${RW_ROOT}/ext/fcl/libccd/src")
        SET(CCD_LIBRARY "ccd")
        SET(CCD_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
        set(CCD_LIBRARIES "${CCD_LIBRARY}")
        SET(CCD_FOUND True)
    endif()
endif()

add_subdirectory(fcl)
