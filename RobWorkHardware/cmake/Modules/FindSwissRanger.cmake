
FIND_PATH(SWISSRANGER_INCLUDE_DIR "swissranger/linux/swissranger.h"
          PATHS
           /usr/include
           /include
          )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SDH_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SwissRanger DEFAULT_MSG 
                                  SWISSRANGER_INCLUDE_DIR)

mark_as_advanced(SWISSRANGER_INCLUDE_DIR)
