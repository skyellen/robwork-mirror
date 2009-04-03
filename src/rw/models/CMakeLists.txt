SET(FILES_CPP
  Accessor.cpp
  Models.cpp
  BasicDevice.cpp
  BasicDeviceJacobian.cpp
  Cartesian6DOFDevice.cpp
  Conveyor.cpp
#  ConveyorBelt.cpp
  ConveyorItem.cpp
  ConveyorSegment.cpp
  Device.cpp
  DeviceJacobian.cpp
  JointDevice.cpp
  FixedJoint.cpp
  JacobianUtil.cpp
  Joint.cpp
  MobileDevice.cpp
  TreeDevice.cpp
  CompositeDevice.cpp
  ParallelDevice.cpp
  ParallelLeg.cpp
  PassivePrismaticFrame.cpp
  PassiveRevoluteFrame.cpp
  PrismaticJoint.cpp
  RevoluteJoint.cpp
  SerialDevice.cpp
  RigidBodyInfo.cpp  
  WorkCell.cpp
  TimeMetricUtil.cpp
  DrawableModelInfo.cpp
  CollisionModelInfo.cpp
)

SET(FILES_HPP
  Accessor.hpp
  Models.hpp
  BasicDevice.hpp
  BasicDeviceJacobian.hpp
  Cartesian6DOFDevice.hpp
  Conveyor.hpp
#  ConveyorBelt.hpp
  ConveyorItem.hpp
  ConveyorSegment.hpp
  Device.hpp
  DeviceJacobian.hpp
  JointDevice.hpp
  FixedJoint.hpp
  JacobianUtil.hpp
  Joint.hpp
  MobileDevice.hpp
  TreeDevice.hpp
  CompositeDevice.hpp
  ParallelDevice.hpp
  ParallelLeg.hpp
  PassivePrismaticFrame.hpp
  PassiveRevoluteFrame.hpp
  PrismaticJoint.hpp
  RevoluteJoint.hpp
  SerialDevice.hpp
  RigidBodyInfo.hpp  
  WorkCell.hpp
  TimeMetricUtil.hpp
  DrawableModelInfo.hpp
  CollisionModelInfo.hpp
)

SOURCE_GROUP(models FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${MODELS_FILES_HPP} DESTINATION "include/rw/sensor")
