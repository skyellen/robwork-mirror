SET(FILES_CPP
  Camera.cpp
  CameraListener.cpp
  Image.cpp
  Sensor.cpp
  TactileArray.cpp
  ImageUtil.cpp
)

SET(FILES_HPP
  Camera.hpp
  CameraListener.hpp
  Image.hpp
  Sensor.hpp
  TactileArray.hpp
  ImageUtil.hpp
)

SOURCE_GROUP(sensor FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${SENSOR_FILES_HPP} DESTINATION "include/rw/sensor")
