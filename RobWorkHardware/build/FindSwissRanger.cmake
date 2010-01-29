

   find_library(CMU1394 1394camera.lib ) #  1394camera.dll 1394camera.dll 1394camera.dll
   if (CMU1394)
 message(STATUS "CMU1394 enabled! 1394camera.lib - found!")
 set(CameraFile ./CMU1394Camera.cpp)
 include_directories(${Ext}/dcam/win/cmu1394)
   else ()
 message(STATUS "CMU1394 disabled! 1394camera.lib - not found!")
   endif ()