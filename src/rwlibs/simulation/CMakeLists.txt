set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.

add_library(
  rw_simulation
  ${RW_SENSORS_USER_FILES}
  ${CameraFile}
  ./camera/FrameGrabber.cpp
  ./camera/GLFrameGrabber.cpp
  ./camera/VirtualCamera.cpp
  ./scanner/GLScanner3D.cpp
  ./scanner/GLScanner2D.cpp
  ./scanner/GLScanner1D.cpp
  ./tactile/CDTactileArray.cpp
)
