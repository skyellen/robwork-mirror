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


#include "FTSensorModel.hpp"

using namespace rw::sensor;

FTSensorModel::FTSensorModel(const std::string& name, rw::kinematics::Frame* frame, const std::string& desc):
    SensorModel(name,frame, desc)
{
	add(_sdata);
}

FTSensorModel::~FTSensorModel(){}

rw::math::Wrench6D<> FTSensorModel::getMaxWrench() const
{
	return _maxWrench;
}

void FTSensorModel::setMaxWrench(const rw::math::Wrench6D<>& max){
	_maxWrench = max;
}

rw::math::Vector3D<> FTSensorModel::getMaxForce() const
{
	return _maxWrench.force();
}

rw::math::Vector3D<> FTSensorModel::getMaxTorque() const
{
	return _maxWrench.torque();
}

void FTSensorModel::setWrench(const rw::math::Wrench6D<>& wrench, const rw::kinematics::State& state){
	_sdata.get(state) = wrench;
}

rw::math::Vector3D<> FTSensorModel::getForce(const rw::kinematics::State& state)  const{
	return _sdata.get(state).force();
}

void FTSensorModel::setForce(const rw::math::Vector3D<>& force, const rw::kinematics::State& state){
	_sdata.get(state).setForce( force );
}

rw::math::Vector3D<> FTSensorModel::getTorque(const rw::kinematics::State& state) const{
	return _sdata.get(state).torque();
}

void FTSensorModel::setTorque(const rw::math::Vector3D<>& torque, const rw::kinematics::State& state){
	_sdata.get(state).setTorque( torque );
}


rw::math::Transform3D<> FTSensorModel::getTransform() const{
	return _offset;
}

void FTSensorModel::setTransform(const rw::math::Transform3D<>& t3d){
	_offset = t3d;
}
