/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/
/**
 * @file rw/models.hpp
 *
 * this file includes all header files from the models namespace
 */

#ifndef RW_MODELS_HPP_
#define RW_MODELS_HPP_

//#include "./models/BasicDevice.hpp"
//#include "./models/BasicDeviceJacobian.hpp"
//#include "./models/Cartesian6DOFDevice.hpp"
//#include "./models/Conveyor.hpp"
//#include "./models/ConveyorBelt.hpp"
//#include "./models/ConveyorItem.hpp"
//#include "./models/ConveyorSegment.hpp"
//#include "./models/DeviceJacobian.hpp"
//#include "./models/PassivePrismaticFrame.hpp"
//#include "./models/TimeMetricUtil.hpp"

#include "./models/CompositeDevice.hpp"
#include "./models/Device.hpp"
#include "./models/VirtualJoint.hpp"
#include "./models/JacobianUtil.hpp"
#include "./models/JacobianCalculator.hpp"
#include "./models/Joint.hpp"
#include "./models/JointDevice.hpp"
#include "./models/JointDeviceJacobianCalculator.hpp"
#include "./models/MobileDevice.hpp"
#include "./models/Models.hpp"
#include "./models/ParallelDevice.hpp"
#include "./models/ParallelLeg.hpp"
#include "./models/DependentRevoluteJoint.hpp"
#include "./models/DependentPrismaticJoint.hpp"
#include "./models/PrismaticJoint.hpp"
#include "./models/RevoluteJoint.hpp"
#include "./models/RigidBodyInfo.hpp"
#include "./models/SerialDevice.hpp"
#include "./models/TreeDevice.hpp"
#include "./models/WorkCell.hpp"
#include "./models/VirtualJoint.hpp"
#include "./models/Object.hpp"
#include "./models/RigidObject.hpp"
#include "./models/DeformableObject.hpp"
#include "./models/DHParameterSet.hpp"

#endif /* MODELS_HPP_ */
