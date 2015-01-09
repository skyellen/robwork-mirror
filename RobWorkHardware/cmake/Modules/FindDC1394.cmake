
# CMake module to search for dc1394 library
#
# If it's found it sets DC1394_FOUND to TRUE
# and following variables are set:
#    DC1394_INCLUDE_DIR
#    DC1394_LIBRARY

FIND_PATH (DC1394_INCLUDE_DIR dc1394/dc1394.h
   # TODO paths?
   /usr/include 
   /include
)

FIND_LIBRARY(DC1394_LIBRARY NAMES dc1394)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SDH_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(DC1394 DEFAULT_MSG 
                                  DC1394_LIBRARY DC1394_INCLUDE_DIR)

mark_as_advanced(DC1394_LIBRARY DC1394_INCLUDE_DIR)


