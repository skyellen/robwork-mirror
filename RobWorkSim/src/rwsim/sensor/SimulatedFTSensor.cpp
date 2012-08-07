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

#include "SimulatedFTSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::sensor;
using namespace rwsim;


namespace {

    class FTSensorWrapper: public rw::sensor::FTSensor {
    private:
        SimulatedFTSensor* _sensor;
    public:
        FTSensorWrapper(SimulatedFTSensor* sensor, Frame* bframe, const std::string& name):
            FTSensor(name),
            _sensor(sensor)
        {
            this->attachTo(bframe);
        }

        double getMaxForce(){ return _sensor->getMaxForce(); };
        double getMaxTorque(){ return _sensor->getMaxTorque(); };
        rw::math::Transform3D<> getTransform(){ return _sensor->getTransform(); };

        void acquire(){ _sensor->acquire(); }
        rw::math::Vector3D<> getForce(){ return _sensor->getForce(); };
        rw::math::Vector3D<> getTorque(){ return _sensor->getTorque(); };
    };


}

/*
SimulatedFTSensor::SimulatedFTSensor(const std::string& name, dynamics::Body *body):
    TactileMultiAxisSensor(name, "TactileMultiAxisSensor"),_body(body)
{
	this->attachTo( body->getBodyFrame() );
}
*/
SimulatedFTSensor::SimulatedFTSensor(const std::string& name,
                                     dynamics::Body::Ptr body,
                                     dynamics::Body::Ptr body1,
                                     rw::kinematics::Frame* frame):
        SimulatedTactileSensor(name),
    _body(body),_body1(body1)
{
    _sframe = _body1->getBodyFrame();
    if(frame!=NULL){
        _sframe = frame;
    }
    _ftsensorWrapper = rw::common::ownedPtr( new FTSensorWrapper(this, _sframe, "SimulatedFTSensor") );
}

rw::math::Transform3D<> SimulatedFTSensor::getTransform(){
	return _transform;
}

rw::math::Vector3D<> SimulatedFTSensor::getForce(){
	return _force;
}

rw::math::Vector3D<> SimulatedFTSensor::getTorque(){
	return _force;
}

void SimulatedFTSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& cnormal,
               rw::kinematics::State& state,
               dynamics::Body::Ptr body)
{
	addForce(_bTw*point, _bTw.R()*force, _bTw.R()*cnormal, state, body);
}

void SimulatedFTSensor::addForce(const rw::math::Vector3D<>& point,
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& cnormal,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{
    _forceTmp += force;
    _torqueTmp += cross(point, force);
}

void SimulatedFTSensor::addWrenchToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{
    _forceTmp += force;
    _torqueTmp += torque;
};

void SimulatedFTSensor::addWrenchWToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{
    _forceTmp += _bTw.R() * force;
    _torqueTmp += _bTw.R() * torque;
};


void SimulatedFTSensor::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
	// update aux variables
	_wTb = Kinematics::worldTframe( _body1->getBodyFrame(), state);
	_fTb = Kinematics::frameTframe( getSensorFrame(), _body1->getBodyFrame(), state);
	_bTw = inverse(_wTb);
	_force = -  (_fTb.R() * _forceTmp );
    _torque = - (_fTb.R() * (_torqueTmp - cross(_fTb.P(), _forceTmp)) );
    _forceTmp = Vector3D<>();
    _torqueTmp = Vector3D<>();

    Vector3D<> wTforce = _wTb.R()*_force;
    Vector3D<> wTtorque = _wTb.R()*_torque;

    // testing the output:
    //std::cout << info.time << "\t" << wTforce[0] << "\t" << wTforce[1] << "\t" << wTforce[2] << "\t"
    //        << wTtorque[0] << "\t" << wTtorque[1] << "\t" << wTtorque[2] << "\n";

    //std::cout << "Force : " << _wTb.R()*_force << std::endl;
    //std::cout << "Torque: " << _wTb.R()*_force << std::endl;
}

void SimulatedFTSensor::reset(const rw::kinematics::State& state){
	_wTb = Kinematics::worldTframe( _body1->getBodyFrame(), state);
    _fTb = Kinematics::frameTframe( getSensorFrame(), _body1->getBodyFrame(), state);
	_bTw = inverse(_wTb);

	_forceTmp = Vector3D<>();
	_torqueTmp = Vector3D<>();
	_force = Vector3D<>();
	_torque = Vector3D<>();
}




