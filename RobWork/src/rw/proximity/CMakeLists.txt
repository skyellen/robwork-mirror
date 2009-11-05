SET(FILES_CPP
  Proximity.cpp
  CollisionStrategy.cpp
  CollisionSetup.cpp
  CollisionToleranceStrategy.cpp
  ProximityStrategy.cpp
  CollisionDetector.cpp
  DistanceStrategy.cpp
  DistanceToleranceStrategy.cpp
  DistanceCalculator.cpp
  ProximityModel.cpp
)

SET(FILES_HPP
  Proximity.hpp
  CollisionStrategy.hpp
  CollisionSetup.hpp
  CollisionToleranceStrategy.hpp
  ProximityStrategy.hpp
  CollisionDetector.hpp
  DistanceStrategy.hpp
  DistanceToleranceStrategy.hpp
  DistanceCalculator.hpp
  ProximityModel.hpp
)

SOURCE_GROUP(proximity FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${PROXIMITY_FILES_HPP} DESTINATION "include/rw/proximity/")
