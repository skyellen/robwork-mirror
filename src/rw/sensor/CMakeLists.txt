SET(CURRENT_LIB_NAME "SENSOR_FILES")

SET(SENSOR_FILES_CPP
  Camera.cpp
  CameraListener.cpp
  Image.cpp
  Sensor.cpp
  TactileArray.cpp
  ImageUtil.cpp
}

SET(SENSOR_FILES_HPP
  Camera.hpp
  CameraListener.hpp
  Image.hpp
  Sensor.hpp
  TactileArray.hpp
  ImageUtil.hpp
}

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SENSOR_FILES_CPP})
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SENSOR_FILES_HPP})

INSTALL(FILES ${RwSrcFiles} ${RwHeaderFiles} DESTINATION include/rw)
INSTALL(FILES "robwork.hpp;use_robwork_namespace.hpp" DESTINATION "include/rw")
