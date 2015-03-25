SET(FILES_CPP
  SensorModel.cpp
  CameraModel.cpp
  
  FTSensorModel.cpp
  Scanner2DModel.cpp
  Scanner25DModel.cpp
  StereoCameraModel.cpp
  RGBDCameraModel.cpp
  TactileArrayModel.cpp
  
  Sensor.cpp
  
  Camera.cpp
  StereoCameraModel.cpp
  CameraFirewire.cpp
  CameraListener.cpp
  Image.cpp
  ImageUtil.cpp
  TactileArray.cpp
  TactileArrayUtil.cpp
  FTSensor.cpp
  Scanner.cpp
  Scanner1D.cpp
  Scanner2D.cpp
  Scanner25D.cpp
  Contact2D.cpp
  Contact3D.cpp
  
)

SET(FILES_HPP
  Sensor.hpp
  
  Camera.hpp
  StereoCameraModel.hpp
  CameraFirewire.hpp
  CameraListener.hpp
  Image.hpp
  ImageUtil.hpp
  
  TactileArray.hpp
  TactileArrayUtil.hpp
  FTSensor.hpp

  Scanner.hpp
  Scanner1D.hpp
  Scanner2D.hpp
  Scanner25D.hpp

  Contact2D.hpp
  Contact3D.hpp
)

SOURCE_GROUP(sensor FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${SENSOR_FILES_HPP} DESTINATION "include/rw/sensor")
