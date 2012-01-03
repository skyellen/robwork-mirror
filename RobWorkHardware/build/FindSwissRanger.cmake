
FIND_PATH(SWISSRANGER_INCLUDE_DIR "swissranger/linux/swissranger.h"
          PATHS
           /usr/include
           /include
          )
SET(SWISSRANGER_FOUND false)
if (SWISSRANGER_INCLUDE_DIR)
    #message(STATUS "CMU1394 enabled! 1394camera.lib - found!")
    SET(SWISSRANGER_FOUND true)
else ()
    #message(STATUS "CMU1394 disabled! 1394camera.lib - not found!")
endif ()

