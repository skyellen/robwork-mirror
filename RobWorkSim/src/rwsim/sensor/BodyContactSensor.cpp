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

#include "BodyContactSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::sensor;
using namespace rwsim;

BodyContactSensor::BodyContactSensor(const std::string& name, rw::kinematics::Frame* frame):
    Sensor(name,"BodyContactSensor")
{
	this->attachTo(frame);
}

BodyContactSensor::~BodyContactSensor(){}

void BodyContactSensor::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){

	 //if(_contactsTmp.size()>0)
     //   std::cout << "Nr contacts in update: " << _contactsTmp.size() << std::endl;
     _contacts = _contactsTmp;
     _contactsTmp.clear();

     _bodies = _bodiesTmp;
     _bodiesTmp.clear();

     //std::cout << "Sensor Forces: ";
     //BOOST_FOREACH(Contact3D& c, _contacts){
     //    std::cout << "--" <<  c.normalForce << "\n";
     //}

     // update aux variables
     _wTf = Kinematics::worldTframe( getFrame(), state);
     _fTw = inverse(_wTf);
}


void BodyContactSensor::reset(const rw::kinematics::State& state){
    _contacts.clear();
    _bodies.clear();
    _wTf = Kinematics::worldTframe( getFrame(), state);
    _fTw = inverse(_wTf);
}

rw::sensor::Sensor* BodyContactSensor::getSensor(){
    return this;
}

void BodyContactSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& snormal,
               rw::kinematics::State& state,
               dynamics::Body *body)
{
    addForce(_fTw*point, _fTw.R()*force, _fTw.R()*snormal, state, body);
}

void BodyContactSensor::addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& snormal,
                  rw::kinematics::State& state,
                  dynamics::Body *body)
{
    //if(body!=NULL)
    std::cout << "addForce("<< point << force << snormal << std::endl;

    _bodiesTmp.push_back( body );
    _contactsTmp.push_back( Contact3D(point, snormal, force)  );
}
