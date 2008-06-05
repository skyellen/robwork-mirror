set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.
if (DEFINED WIN32)

# This stuff doesn't work as intended.
#   find_library(CMU1394 1394camera.lib) #  1394camera.dll 1394camera.dll 1394camera.dll
#   if (CMU1394)
#     message("-- Looking for 1394camera.lib - found")
#     set(CameraFile ./camera/CMU1394Camera.cpp)
#     include_directories(${Ext}/dcam/win/cmu1394)
#   else ()
#     message("-- Looking for 1394camera.lib - not found")
#   endif ()

  set(CameraFile ./camera/CMU1394Camera.cpp)
else ()
  find_library(DC1394 dc1394_control)
  find_library(RAW1394 raw1394)
  if (DC1394 AND RAW1394)
    message("-- Looking for libdc1394 and libraw1394 - found")
    set(CameraFile ./camera/DC1394Camera.cpp)
    include_directories(${Ext}/dcam/linux)
  else ()
    message("-- Looking for libdc1394 and libraw1394 - not found")
  endif ()
endif ()

add_library(
  rw_sensors
  ${RW_SENSORS_USER_FILES}
  ${CameraFile}
  ./camera/FrameGrabber.cpp
  ./camera/GLFrameGrabber.cpp
  ./camera/VirtualCamera.cpp
  ./swissranger/SRCalibrationData.cpp
  ./swissranger/SwissRanger.cpp
  ./tactile/DSACON32.cpp
  ./tactile/TactileMatrix.cpp)
