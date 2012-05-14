SET(FILES_CPP
  Sensor.cpp
  
  Camera.cpp
  StereoCamera.cpp
  CameraFirewire.cpp
  CameraListener.cpp
  Image.cpp
  ImageUtil.cpp
  
  TactileArray.cpp
  TactileMultiAxisSensor.cpp
  TactileArrayUtil.cpp
  
  Scanner.cpp
  Scanner1D.cpp
  Scanner2D.cpp
  Scan2D.cpp
  Scanner25D.cpp
  Image25D.cpp
  
  Contact2D.cpp
  Contact3D.cpp
)

SET(FILES_HPP
  Camera.hpp
  StereoCamera.hpp
  CameraFirewire.hpp
  CameraListener.hpp
  Image.hpp
  Sensor.hpp
  TactileArray.hpp
  ImageUtil.hpp
  Contact2D.hpp
  Contact3D.hpp
  TactileArrayUtil.hpp
)

SOURCE_GROUP(sensor FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${SENSOR_FILES_HPP} DESTINATION "include/rw/sensor")
