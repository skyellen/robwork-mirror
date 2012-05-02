# - Try to find SDH
# Once done this will define
#  SDH_FOUND - System has SDH
#  SDH_INCLUDE_DIRS - The SDH include directories
#  SDH_LIBRARY - The library needed to use SDH

SET(SDH_INCLUDE_DIR_HINT "${SDH_INCLUDE_DIR}")
UNSET(SDH_INCLUDE_DIR)

FIND_PATH(SDH_INCLUDE_DIR
   NAMES
      sdh/sdh.h
   # TODO paths?
   HINTS
      "${SDH_INCLUDE_DIR_HINT}"
      "${RWHW_ROOT}/ext/sdh2/"
      /usr/include
      /include
)

SET(SDH_LIB_DIR_HINT ${SDH_LIB_DIR})
UNSET(SDH_LIB_DIR)

FIND_LIBRARY(SDH_LIBRARY
   NAMES
      sdh
   HINTS
      "${SDH_LIB_DIR_HINT}"
      "${RWHW_ROOT}/ext/sdh2/libs/"
)

#MESSAGE("SDH LIB ${SDH_LIBRARY}")
#MESSAGE("SDH INCLUDE ${SDH_INCLUDE_DIR}")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SDH_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SDH DEFAULT_MSG
                                  SDH_LIBRARY SDH_INCLUDE_DIR)

mark_as_advanced(SDH_INCLUDE_DIR SDH_LIBRARY)
