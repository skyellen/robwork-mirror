

   link_directories(${Ext}/dcam_c)
   #find_library(CMU1394 ${Ext}/dcam_c/1394Camera_c.lib ) #  1394camera.dll 1394camera.dll 1394camera.dll
   #if (CMU1394)
     set(CameraFile ./CMU1394Camera_c.cpp)
     include_directories( ${Ext}/dcam_c/ )
     include_directories(${Ext}/dcam/win/cmu1394/)
     link_libraries(1394Camera_c.lib)
   #else ()
   #  message(STATUS "CMU1394C disabled! 1394camera_c.lib - not found!")
   #endif ()