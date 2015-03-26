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

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::sensor;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;

namespace {

    class FTSensorWrapper: public FTSensor {
    private:
        SimulatedFTSensor* _sensor;
    public:
        FTSensorWrapper(SimulatedFTSensor* sensor, Frame* bframe, const std::string& name):
            FTSensor(name),
            _sensor(sensor)
        {
            //this->attachTo(bframe);
        }

        double getMaxForce(){ return _sensor->getMaxForce().normInf(); };
        double getMaxTorque(){ return _sensor->getMaxTorque().normInf(); };
        Transform3D<> getTransform(){ return _sensor->getTransform(); };

        void acquire(){ _sensor->acquire(); }
        Vector3D<> getForce(){ /*return _sensor->getForce();*/ };
        Vector3D<> getTorque(){ /*return _sensor->getTorque();*/ };
    };

    Frame* getFrameFromBodyOr(Frame* frame, Body::Ptr b){
    	if(frame==NULL)
    		return b->getBodyFrame();
    	return frame;
    }
}

/*
SimulatedFTSensor::SimulatedFTSensor(const std::string& name, dynamics::Body *body):
    TactileMultiAxisSensor(name, "TactileMultiAxisSensor"),_body(body)
{
	this->attachTo( body->getBodyFrame() );
}
*/


SimulatedFTSensor::SimulatedFTSensor(const std::string& name,
                                     Body::Ptr body,
                                     Body::Ptr body1,
                                     Frame* frame):
        SimulatedTactileSensor(rw::common::ownedPtr( new FTSensorModel(name,getFrameFromBodyOr(frame,body1),"SimulatedFtSensor"))),
    _body(body),_body1(body1)
{
	_ftmodel = getSensorModel().cast<FTSensorModel>();

    _sframe = _body1->getBodyFrame();
    if(frame!=NULL){
        _sframe = frame;
    }
    _ftsensorWrapper = rw::common::ownedPtr( new FTSensorWrapper(this, _sframe, "SimulatedFTSensor") );
    add(_sdata);
}

SimulatedFTSensor::~SimulatedFTSensor(){

}

Transform3D<> SimulatedFTSensor::getTransform() const
{
	return _ftmodel->getTransform();
}

Vector3D<> SimulatedFTSensor::getForce(const State& state) const
{
	return _ftmodel->getForce(state);
}

Vector3D<> SimulatedFTSensor::getTorque(const State& state) const
{
	return _ftmodel->getTorque(state);
}

void SimulatedFTSensor::addForceW(const Vector3D<>& point,
               const Vector3D<>& force,
               const Vector3D<>& cnormal,
               State& state,
               Body::Ptr body)
{
	const FTStateData &data = _sdata.get(state);
	addForce(data._bTw*point, data._bTw.R()*force, data._bTw.R()*cnormal, state, body);
}

void SimulatedFTSensor::addForce(const Vector3D<>& point,
              const Vector3D<>& force,
              const Vector3D<>& cnormal,
              State& state,
              Body::Ptr body)
{
	FTStateData &data = _sdata.get(state);
	data._forceTmp += force;
	data._torqueTmp += cross(point, force);
}

void SimulatedFTSensor::addWrenchToCOM(
              const Vector3D<>& force,
              const Vector3D<>& torque,
              State& state,
              Body::Ptr body)
{
	FTStateData &data = _sdata.get(state);
    data._forceTmp += force;
    data._torqueTmp += torque;
};

void SimulatedFTSensor::addWrenchWToCOM(
              const Vector3D<>& force,
              const Vector3D<>& torque,
              State& state,
              Body::Ptr body)
{
	FTStateData &data = _sdata.get(state);
	data._forceTmp += data._bTw.R() * force;
	data._torqueTmp += data._bTw.R() * torque;
};


void SimulatedFTSensor::update(const Simulator::UpdateInfo& info, State& state){
	// update aux variables
	FTStateData &data = _sdata.get(state);
	data._wTb = Kinematics::worldTframe( _body1->getBodyFrame(), state);
	data._fTb = Kinematics::frameTframe( getSensorFrame(), _body1->getBodyFrame(), state);
	data._bTw = inverse(data._wTb);

	data._force = -  (data._fTb.R() * data._forceTmp );
	data._torque = - (data._fTb.R() * (data._torqueTmp - cross(data._fTb.P(), data._forceTmp)) );

	data._forceTmp = Vector3D<>();
	data._torqueTmp = Vector3D<>();

    _ftmodel->setForce(data._force, state);
    _ftmodel->setTorque(data._torque, state);


    //Vector3D<> wTforce = _wTb.R()*_force;
    //Vector3D<> wTtorque = _wTb.R()*_torque;

    // testing the output:
    /*std::cout << info.time << "\t" << wTforce[0] << "\t" << wTforce[1] << "\t" << wTforce[2] << "\t"
            << wTtorque[0] << "\t" << wTtorque[1] << "\t" << wTtorque[2] << "\n";
            */

    //std::cout << "Force : " << _wTb.R()*_force << std::endl;
    //std::cout << "Torque: " << _wTb.R()*_force << std::endl;
}

void SimulatedFTSensor::reset(const State& state){
	FTStateData &data = _sdata.get(state);
	data._wTb = Kinematics::worldTframe( _body1->getBodyFrame(), state);
	data._fTb = Kinematics::frameTframe( getSensorFrame(), _body1->getBodyFrame(), state);
	data._bTw = inverse(data._wTb);

	data._forceTmp = Vector3D<>();
	data._torqueTmp = Vector3D<>();
	data._force = Vector3D<>();
	data._torque = Vector3D<>();
}

FTSensor::Ptr SimulatedFTSensor::getFTSensor(State& state)
{
	// TODO the handle should come from the state cache
	return _ftsensorWrapper;
};
