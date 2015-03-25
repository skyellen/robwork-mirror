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

namespace {

}

BodyContactSensor::BodyContactSensor(const std::string& name, rw::kinematics::Frame* frame):
        SimulatedTactileSensor( rw::common::ownedPtr( new SensorModel(name,frame,"Direct mapped tactile body contact sensor."))),
        _sdata(1, rw::common::ownedPtr( new ClassState()).cast<rw::kinematics::StateCache>())
{
	add(_sdata);
}

BodyContactSensor::~BodyContactSensor(){}

void BodyContactSensor::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
	RW_WARN("");

	if( !isRegistered() )
		RW_WARN("BOSY NOT REGISTERED IN STATE YET...");

	//(if(_contactsTmp.size()>0)
     //   std::cout << "Nr contacts in update: " << _contactsTmp.size() << std::endl;
	ClassState* cstate = _sdata.getStateCache<ClassState>(state);
	if(cstate==NULL)
		RW_WARN("CSTATE IS NULL");

	RW_WARN("");
	cstate->_contacts = cstate->_contactsTmp;
	cstate->_contactsTmp.clear();
	RW_WARN("");
	cstate->_bodies = cstate->_bodiesTmp;
	cstate->_bodiesTmp.clear();
	RW_WARN("");
     //std::cout << "Sensor Forces: ";
     //BOOST_FOREACH(Contact3D& c, _contacts){
     //    std::cout << "--" <<  c.normalForce << "\n";
     //}

     // update aux variables
     _wTf = Kinematics::worldTframe( getFrame(), state);
     _fTw = inverse(_wTf);
     RW_WARN("");
}


void BodyContactSensor::reset(const rw::kinematics::State& state){
	ClassState* cstate = _sdata.getStateCache<ClassState>(state);
	cstate->_contacts.clear();
	cstate->_bodies.clear();
    _wTf = Kinematics::worldTframe( getFrame(), state);
    _fTw = inverse(_wTf);
}

rw::sensor::Sensor::Ptr BodyContactSensor::getSensor(){
    return NULL;
}

void BodyContactSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& snormal,
               rw::kinematics::State& state,
               dynamics::Body::Ptr body)
{
    addForce(_fTw*point, _fTw.R()*force, _fTw.R()*snormal, state, body);
}

void BodyContactSensor::addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& snormal,
                  rw::kinematics::State& state,
                  dynamics::Body::Ptr body)
{
    //if(body!=NULL)
    //std::cout << "addForce("<< point << force << snormal << std::endl;
	ClassState* cstate = _sdata.getStateCache<ClassState>(state);
	cstate->_bodiesTmp.push_back( body );
	cstate->_contactsTmp.push_back( Contact3D(point, snormal, force)  );
}
