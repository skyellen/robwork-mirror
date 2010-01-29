

  find_library(DC1394 dc1394_control)
  find_library(RAW1394 raw1394)
  if (DC1394 AND RAW1394)
    #message(STATUS "DC1394 enabled! libdc1394 and libraw1394 - found!")
    set(CameraFile ./DC1394Camera.cpp)
    include_directories(${Ext}/dcam/linux)
  else ()
    #message(STATUS "DC1394 disabled! libdc1394 or libraw1394 - not found!")
  endif ()