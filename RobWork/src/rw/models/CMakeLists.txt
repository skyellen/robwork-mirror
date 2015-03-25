SET(FILES_CPP
  Models.cpp
  #BasicDevice.cpp
  #BasicDeviceJacobian.cpp
  #Cartesian6DOFDevice.cpp
  #Conveyor.cpp
  #ConveyorBelt.cpp
  #ConveyorSegment.cpp
  #DeviceJacobian.cpp
  #TimeMetricUtil.cpp
  ConveyorItem.cpp
  ControllerModel.cpp
  Device.cpp
  JointDevice.cpp
  VirtualJoint.cpp
  JacobianUtil.cpp
  Joint.cpp
  JacobianCalculator.cpp
  JointDeviceJacobianCalculator.cpp  
  #DeviceJacobianCalculator.cpp
  MobileDevice.cpp
  TreeDevice.cpp
  CompositeDevice.cpp
  CompositeJointDevice.cpp
  ParallelDevice.cpp
  ParallelLeg.cpp
  DependentJoint.cpp
  DependentPrismaticJoint.cpp
  DependentRevoluteJoint.cpp
  PrismaticJoint.cpp
  RevoluteJoint.cpp
  SerialDevice.cpp
  RigidBodyInfo.cpp  
  WorkCell.cpp
  DHParameterSet.cpp
  Object.cpp
  RigidObject.cpp
  DeformableObject.cpp
  SE3Device.cpp
  
  
)

SET(FILES_HPP
  # BasicDevice.hpp
  # BasicDeviceJacobian.hpp
  # Cartesian6DOFDevice.hpp
  # Conveyor.hpp
  # ConveyorBelt.hpp
  # ConveyorSegment.hpp
  # DeviceJacobian.hpp 
  #TimeMetricUtil.hpp
  ControllerModel.hpp
  Models.hpp
  ConveyorItem.hpp
  Device.hpp
  JointDevice.hpp
  VirtualJoint.hpp
  JacobianUtil.hpp
  Joint.hpp  
  JacobianCalculator.hpp
  JointDeviceJacobianCalculator.hpp    
  MobileDevice.hpp
  TreeDevice.hpp
  CompositeDevice.hpp
  ParallelDevice.hpp
  ParallelLeg.hpp
  DependentJoint.hpp
  DependentPrismaticJoint.hpp
  DependentRevoluteJoint.hpp
  PrismaticJoint.hpp
  RevoluteJoint.hpp
  SerialDevice.hpp
  RigidBodyInfo.hpp  
  WorkCell.hpp
  DHParameterSet.hpp
  Object.hpp
  SE3Device.hpp
)

SOURCE_GROUP(models FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${MODELS_FILES_HPP} DESTINATION "include/rw/sensor")
