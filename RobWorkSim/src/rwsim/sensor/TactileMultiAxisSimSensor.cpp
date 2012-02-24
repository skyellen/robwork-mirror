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

#include "TactileMultiAxisSimSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::sensor;
using namespace rwsim;
/*
TactileMultiAxisSimSensor::TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body):
    TactileMultiAxisSensor(name, "TactileMultiAxisSensor"),_body(body)
{
	this->attachTo( body->getBodyFrame() );
}
*/
TactileMultiAxisSimSensor::TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body, dynamics::Body *body1):
    TactileMultiAxisSensor(name, "TactileMultiAxisSensor"),_body(body),_body1(body1)
{
    this->attachTo( body1->getBodyFrame() );
}

rw::math::Transform3D<> TactileMultiAxisSimSensor::getTransform(){
	return _transform;
}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getForce(){
	return _force;
}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getTorque(){
	return _force;
}

void TactileMultiAxisSimSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& cnormal,
               rw::kinematics::State& state,
               dynamics::Body *body)
{
	addForce(_fTw*point, _fTw.R()*force, _fTw.R()*cnormal, state, body);
}

void TactileMultiAxisSimSensor::addForce(const rw::math::Vector3D<>& point,
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& cnormal,
              rw::kinematics::State& state,
              dynamics::Body *body)
{
    _forceTmp += force;
    _torqueTmp += cross(point, force);
}

void TactileMultiAxisSimSensor::addWrenchToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body *body)
{
    _forceTmp += force;
    _torqueTmp += torque;
};

void TactileMultiAxisSimSensor::addWrenchWToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body *body)
{
    _forceTmp += _fTw.R() * force;
    _torqueTmp += _fTw.R() * torque;
};


void TactileMultiAxisSimSensor::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
	// update aux variables
	_wTf = Kinematics::worldTframe( getFrame(), state);
	_fTw = inverse(_wTf);
	_force = _forceTmp;
    _torque = _torqueTmp;
    _forceTmp = Vector3D<>();
    _torqueTmp = Vector3D<>();
}

void TactileMultiAxisSimSensor::reset(const rw::kinematics::State& state){
	_wTf = Kinematics::worldTframe( getFrame(), state);
	_fTw = inverse(_wTf);
	_forceTmp = Vector3D<>();
	_torqueTmp = Vector3D<>();
}


