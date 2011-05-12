
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

IF (DC1394_INCLUDE_DIR AND DC1394_LIBRARY)
    #MESSAGE(STATUS "DC1394: Found!")
    SET (DC1394_FOUND 1)
ELSE()
    #MESSAGE(STATUS "DC1394: Not found!")
ENDIF (DC1394_INCLUDE_DIR AND DC1394_LIBRARY)





 # find_library(DC1394 dc1394_control)
 # find_library(RAW1394 raw1394)
 # if (DC1394 AND RAW1394)
#	SET(DC1394_FOUND 1)
#    #message(STATUS "DC1394 enabled! libdc1394 and libraw1394 - found!")
#    #set(CameraFile ./DC1394Camera.cpp)
#    #include_directories(${Ext}/dcam/linux)
#    Set(DC1394_libs ${DC1394} ${RAW1394})
#    message(STATUS "DC1394_libs = ${DC1394_libs}")
#  else ()
#    #message(STATUS "DC1394 disabled! libdc1394 or libraw1394 - not found!")
#	SET(DC1394_FOUND 0)
#  endif ()
