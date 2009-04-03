set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.

SET(SRC_FILES
  ${RW_SENSORS_USER_FILES}
  ${CameraFile}
  ./camera/FrameGrabber.cpp
  ./camera/GLFrameGrabber.cpp
  ./camera/VirtualCamera.cpp
)

SET(SRC_FILES_HPP
  ./camera/FrameGrabber.hpp
  ./camera/GLFrameGrabber.hpp
  ./camera/VirtualCamera.hpp
)

ADD_LIBRARY(rw_simulation ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_simulation DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)
