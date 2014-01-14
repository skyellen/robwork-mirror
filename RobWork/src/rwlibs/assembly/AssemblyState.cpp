/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "AssemblyState.hpp"
#include <rwlibs/task/Target.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::assembly;
using namespace rwlibs::task;

AssemblyState::AssemblyState():
	contact(false)
{
};

AssemblyState::AssemblyState(CartesianTarget::Ptr target) {
	femaleTmale = target->getValue<Transform3D<> >();
	phase = target->getPropertyMap().get<std::string>("Phase","");
	femaleOffset = target->getPropertyMap().get<Transform3D<> >("FemaleOffset",Transform3D<>());
	maleOffset = target->getPropertyMap().get<Transform3D<> >("MaleOffset",Transform3D<>());
	Vector3D<> force, torque;
	force = target->getPropertyMap().get<Vector3D<> >("FTSensorMaleForce",Vector3D<>());
	torque = target->getPropertyMap().get<Vector3D<> >("FTSensorMaleTorque",Vector3D<>());
	ftSensorMale = Wrench6D<>(force,torque);
	force = target->getPropertyMap().get<Vector3D<> >("FTSensorFemaleForce",Vector3D<>());
	torque = target->getPropertyMap().get<Vector3D<> >("FTSensorFemaleTorque",Vector3D<>());
	ftSensorFemale = Wrench6D<>(force,torque);
	contact = target->getPropertyMap().get<bool>("Contact",false);
}

AssemblyState::~AssemblyState() {
}

CartesianTarget::Ptr AssemblyState::toCartesianTarget(const AssemblyState &state) {
	CartesianTarget::Ptr target = ownedPtr(new CartesianTarget(state.femaleTmale));
	target->getPropertyMap().set<Transform3D<> >("FemaleOffset",state.femaleOffset);
	target->getPropertyMap().set<Transform3D<> >("MaleOffset",state.maleOffset);
	target->getPropertyMap().set<std::string>("Phase",state.phase);
	if (!(state.ftSensorMale.force() == Vector3D<>::zero()))
		target->getPropertyMap().set<Vector3D<> >("FTSensorMaleForce",state.ftSensorMale.force());
	if (!(state.ftSensorMale.torque() == Vector3D<>::zero()))
		target->getPropertyMap().set<Vector3D<> >("FTSensorMaleTorque",state.ftSensorMale.torque());
	if (!(state.ftSensorFemale.force() == Vector3D<>::zero()))
		target->getPropertyMap().set<Vector3D<> >("FTSensorFemaleForce",state.ftSensorFemale.force());
	if (!(state.ftSensorFemale.torque() == Vector3D<>::zero()))
		target->getPropertyMap().set<Vector3D<> >("FTSensorFemaleTorque",state.ftSensorFemale.torque());
	target->getPropertyMap().set<bool>("Contact",state.contact);
	return target;
}
