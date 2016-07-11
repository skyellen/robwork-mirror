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


#include "TargetConfigGraspPolicy.hpp"

#include <rwsim/control/PDController.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rwlibs::control;
using namespace rwsim::util;
using namespace rwsim::control;

TargetConfigGraspPolicy::TargetConfigGraspPolicy(rwsim::dynamics::DynamicDevice* dev):
		_dev(dev)
{
	// add default settings to the property map
	setDefaultSettings();
}


TargetConfigGraspPolicy::~TargetConfigGraspPolicy(){

}

void TargetConfigGraspPolicy::setDefaultSettings(){
	Q initMaskQ( _dev->getModel().getDOF() , 1.0);
	Q initTargetQ( _dev->getModel().getDOF() , 0.0);
	_settings.add<std::string>("Controller","This is the position controller used to reach the target position. Default is PDController.","PDController");
	_settings.add<rw::math::Q>("Mask","This defines which joints to use. 1: set to target config, 0: use initial position (defined by state).",initMaskQ);
	_settings.add<std::string>("Hueristic","Defines how target positions are generated. Valid Options are: SET, BOUNDS and OFFSET", "SET");



}

// inherited from GraspPolicy
void TargetConfigGraspPolicy::reset(const rw::kinematics::State& state){
    // generate
}

rwlibs::simulation::SimulatedController::Ptr TargetConfigGraspPolicy::getController(){
    PDParam pdparam;
    _controller = ownedPtr( new PDController("TargetController", _dev, JointController::POSITION, pdparam, 0.02) );
	return _controller;
}

void TargetConfigGraspPolicy::applySettings(){

}
