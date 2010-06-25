set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.

SET(SRC_FILES
  ${RW_SENSORS_USER_FILES}
  ${CameraFile}
  ./camera/FrameGrabber.cpp
  ./camera/GLFrameGrabber.cpp
  ./camera/SimulatedCamera.cpp
  RWGLFrameBuffer.cpp
  FrameGrabber25D.cpp
  GLFrameGrabber25D.cpp
  KinematicSimulator.cpp
  SimulatedController.cpp
  SimulatedSensor.cpp
  Simulator.cpp
  SimulatedScanner25D.cpp
  SimulatedScanner2D.cpp
)

SET(SRC_FILES_HPP
  ./camera/FrameGrabber.hpp
  ./camera/GLFrameGrabber.hpp
  ./camera/SimulatedCamera.hpp
	RWGLFrameBuffer.hpp
  FrameGrabber25D.hpp
  GLFrameGrabber25D.hpp
  KinematicSimulator.hpp
  SimulatedController.hpp
  SimulatedSensor.hpp
  Simulator.hpp
  SimulatedScanner25D.hpp
  SimulatedScanner2D.hpp
)

ADD_LIBRARY(rw_simulation ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_simulation DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)
