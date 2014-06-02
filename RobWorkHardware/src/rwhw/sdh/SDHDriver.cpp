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

#include "SDHDriver.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/common/Log.hpp>
#include <sdh/sdh.h>

#ifdef RWHW_HAS_ESDCAN
#include <rwhw/can/ESDCAN/ESDCANPort.hpp>
#endif

#include <iostream>
#include <float.h>


using namespace SDH;

using namespace rw::math;
using namespace rwhw;
using namespace rw::common;

namespace {

	std::vector<double> toStdVector(const rw::math::Q& q){
		std::vector<double> vq(q.size()+1);
		for(size_t i=0;i<q.size();i++)
			vq[i] = q(i);
		vq.back() = 0;
		return vq;
	}

	void setStdVector(std::vector<double>& vq, const rw::math::Q& q){
		//RW_ASSERT(q.size()<=vq.size());
		for(size_t i=0;i<q.size();i++)
			vq[i] = q(i);
		vq.back() = 0;
	}

	Q toQ(const std::vector<double>& vq){
		Q q(vq.size()-1);
		for(size_t i=0;i<q.size();i++)
			q[i] = vq[i];
		return q;
	}
}

SDHDriver::SDHDriver():
        _moveCmdTimeout(1.0)
        {
	bool useRadians = true;
	bool useFahrenheit = false;
	int debugLevel = 0;
	_hand = new cSDH(useRadians, useFahrenheit, debugLevel );
	_axes = _hand->all_axes;
	// thumb
	_axes[0] = 3;
	_axes[1] = 4;

	// special
	_axes[2] = 0;

	_axes[3] = 1;
	_axes[4] = 2;

	_axes[5] = 5;
	_axes[6] = 6;

	_axes[7] = 7; // virtual axis, should allways be 0

	_vjointTarget.resize(_axes.size());
	_vjointTargetVel.resize(_axes.size());
	_vjointTmp.resize(_axes.size());
}

SDHDriver::~SDHDriver(){
    delete _hand;
}

bool SDHDriver::connect(const std::string& device, int canBaudRate, double canTimeOut, int id_read, int id_write){
    try {
        _hand->OpenCAN_PEAK( canBaudRate , canTimeOut, id_read, id_write, device.c_str() );
    } catch (cSDHLibraryException* e){
        rw::common::Log::errorLog() << e->what();
        delete e;
        return false;
    }
    initConnection();
    return isConnected();

}

bool SDHDriver::connect(int canNetId, int canBaudRate, double canTimeOut, int id_read, int id_write){
    try {

        _hand->OpenCAN_ESD( canNetId, canBaudRate , canTimeOut, id_read, id_write);

    } catch (cSDHLibraryException* e){
        rw::common::Log::errorLog() << e->what();
        delete e;
        return false;
    }
    initConnection();
    return isConnected();
}

bool SDHDriver::connect( CanPort::Ptr canport, double canTimeOut, int id_read, int id_write){

    #ifdef RWHW_HAS_ESDCAN
        ESDCANPort::Ptr esdport = canport.cast<ESDCANPort>();
        if(esdport==NULL)
            RW_THROW("CAN port must be of type ESDCANPort!");

        NTCAN_HANDLE handle = esdport->getHandle();

        try {
            _hand->OpenCAN_ESD( handle, canTimeOut, id_read, id_write);
        } catch (cSDHLibraryException* e){
            rw::common::Log::errorLog() << e->what();
            delete e;
            return false;
        }
        return initConnection();
	#elif RWHW_HAS_PEAKCAN
        // TODO: implement a peak can interface under can
    #else
        RW_THROW("RobWorkHardware does not seem to be build with ESDCANPort!");
        return false;
    #endif
}

bool SDHDriver::connect(int port, unsigned long baudrate, double timeout, char const* device_format_string ){
    // Open communication to the SDH device via default serial port 0 == "COM1"
    //
    try {
    	if (device_format_string) {
    		_hand->OpenRS232( port, baudrate, timeout, device_format_string);
    	} else {
    		_hand->OpenRS232( port, baudrate, timeout);
    	}
    } catch (cSDHLibraryException* e){
        rw::common::Log::errorLog() << e->what();
        delete e;
        return false;
    }

   return initConnection();
}

bool SDHDriver::initConnection(){
    try {
        _hand->SetVelocityProfile( cSDH::eVP_RAMP );
        _minPos = toQ(_hand->GetAxisMinAngle(_axes));
        _maxPos = toQ(_hand->GetAxisMaxAngle(_axes));
    } catch (cSDHLibraryException* e){
        rw::common::Log::errorLog() << e->what();
        _hand->Close();
        delete e;
        return false;
    }
    //_thread = new boost::thread(boost::bind(&SDHDriver::updateLoop, this));

    return isConnected();
}

//const std::string statesStr[] = {"eAS_IDLE", "eAS_POSITIONING", "eAS_IDLE", "eAS_IDLE", "eAS_IDLE", "eAS_DISABLED", "eAS_IDLE"};
const std::string statesStr[] = {
    "eAS_IDLE",
    "eAS_POSITIONING",         //!< the goal position has not been reached yet
    "eAS_SPEED_MODE",          //!< axis is in speed mode
    "eAS_NOT_INITIALIZED",     //!< axis is not initialized or doesn't exist
    "eAS_CW_BLOCKED",          //!< axis is blocked in counterwise direction
    "eAS_CCW_BLOCKED",         //!< axis is blocked is blocked in against counterwise direction
    "eAS_DISABLED",            //!< axis is disabled
    "eAS_LIMITS_REACHED",      //!< position limits reached and axis stopped
    "eAS_DIMENSION"}; //!< Endmarker and Dimension

void SDHDriver::updateLoop(){
    //while(_hand->IsOpen()){
        std::vector<cSDH::eAxisState> states = _hand->GetAxisActualState(_axes);
        std::cout << "JointState {";
        for(size_t i=0;i<states.size();i++){
            std::cout << statesStr[states[i]] << ",";
        }
        std::cout <<"\n";

      //  TimerUtil::sleepMs(1000);
    //}
}

bool SDHDriver::isConnected(){
	return _hand->IsOpen();
}

void SDHDriver::disconnect(){
	try {
		_hand->Close();
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::moveCmd(bool block){
	try {
		_hand->MoveAxis(_axes, block);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::moveCmd(rw::math::Q target, bool block){
	// adjust the dependent joint
	rw::math::Q tq = Math::clampQ(target,_minPos,_maxPos);
	setStdVector(_vjointTmp, tq);
	try {
		_hand->SetAxisTargetAngle(_axes, _vjointTmp);
		_hand->MoveAxis(_axes, block);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::moveJointCmd(int jointIdx, double target, bool block){
	try {
		_hand->SetAxisTargetAngle(jointIdx, target);
		_hand->MoveAxis(_axes, block);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

bool SDHDriver::waitCmd(double timeout){
	try {
		_hand->WaitAxis(_axes, timeout);
	} catch (cSDHLibraryException* e){
		delete e;
		return false;
	}


	return true;
	// test if target is reached
/*
	std::vector<cSDH::eAxisState> states = _hand->GetAxisActualState(_axes);
	bool posreached = true;
	for(size_t i=0;i<states.size();i++){
		posreached &= (states[i] == cSDH::eAS_IDLE);
	}
	return posreached;
	*/
}

rw::math::Q SDHDriver::getTargetQ(){
	std::vector<double> target;
	try{
		target = _hand->GetAxisTargetAngle(_axes);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return toQ(target);
}

void SDHDriver::setTargetQ(const rw::math::Q& jointPos){
	// adjust the dependent joint
	rw::math::Q tq = Math::clampQ(jointPos,_minPos,_maxPos);
	setStdVector(_vjointTmp, tq);
	try{
		_hand->SetAxisTargetAngle(_axes, _vjointTmp );
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::setTargetQVel(const rw::math::Q& jointVel){
	setStdVector(_vjointTmp, jointVel);
	try{
		_hand->SetAxisTargetVelocity(_axes, _vjointTmp );
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::setTargetQAcc(const rw::math::Q& jointAcc){
	setStdVector(_vjointTmp, jointAcc);
	try{
		_hand->SetAxisTargetAcceleration(_axes, _vjointTmp );
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

void SDHDriver::setTargetQCurrent(const rw::math::Q& jointCurr){
	setStdVector(_vjointTmp, jointCurr);
	try{
		_hand->SetAxisMotorCurrent(_axes, _vjointTmp );
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex << "send value0: " << _vjointTmp.at(0) << " size: " << _vjointTmp.size());
	}
}

rw::math::Q SDHDriver::getQ(){
	std::vector<double> target;
	try{
		target = _hand->GetAxisActualAngle(_axes);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return toQ(target);
}

rw::math::Q SDHDriver::getdQ(){
	std::vector<double> vel;
	try{
		vel = _hand->GetAxisActualVelocity(_axes);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return toQ(vel);
}

rw::math::Q SDHDriver::getQCurrent() {
	std::vector<double> current;
	try{
		current = _hand->GetAxisMotorCurrent(_axes);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
  return toQ(current);
}

void SDHDriver::stop(){
	try{
		_hand->Stop();
		std::vector<double> disableVector = toStdVector(Q(_axes.size()-1,0.0));
		_hand->SetAxisEnable(_axes, disableVector);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

int SDHDriver::getDOF(){
	return _hand->GetNumberOfAxes();
};

void SDHDriver::emergencyStop(){
	try{
		_hand->EmergencyStop();
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
}

std::pair<rw::math::Q,rw::math::Q> SDHDriver::getPosLimits(){
	Q minQ, maxQ;
	try{
		minQ = toQ(_hand->GetAxisMinAngle(_axes));
		maxQ = toQ(_hand->GetAxisMaxAngle(_axes));
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return std::pair<Q,Q>(minQ,maxQ);
}

rw::math::Q SDHDriver::getVelLimits(){
	std::vector<double> vellimits;
	try{
		vellimits = _hand->GetAxisMaxVelocity(_axes) ;
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	//		The max limits are not a valid value to send, but needed to round down!? This works!
	return toQ(vellimits)- rw::math::Q((toQ(vellimits)).size(),0.01);
}

rw::math::Q SDHDriver::getAccLimits(){
	std::vector<double> acclimits;
	try{

		acclimits = _hand->GetAxisMaxAcceleration(_axes);
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	//		The max limits are not a valid value to send, but needed to round down!? This works!
	return toQ(acclimits) - rw::math::Q(acclimits.size(),0.01);
}

rw::math::Q SDHDriver::getCurrentLimits(){
    //std::cout << "getting current limits" << std::endl;
	std::vector<double> current;
	try{
		//current = _hand->GetAxisMotorCurrent(_axes);
		current.resize(_hand->GetNumberOfAxes()+1);
		for(unsigned int i=0; i<current.size(); ++i) {
			current.at(i)=1.0;
		}
		//std::cout << "MAX motor current: " << toQ(current) << std::endl;
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return toQ(current);
}

std::vector<double> SDHDriver::getTemperature() {
    //std::cout << "getting current limits" << std::endl;
	std::vector<double> temp;
	try{
		temp = _hand->GetTemperature( _hand->all_temperature_sensors );
	} catch (cSDHLibraryException* e){
		std::string ex = e->what();
		delete e;
		RW_THROW(ex);
	}
	return temp;
}

void SDHDriver::setJointEnabled(bool enabled){
    try{
        if(enabled)
            _hand->SetAxisEnable(_axes, toStdVector(Q(7,1.0)));
        else
            _hand->SetAxisEnable(_axes, toStdVector(Q(7,0.0)));
    } catch (cSDHLibraryException* e){
        std::string ex = e->what();
        delete e;
        RW_THROW(ex);
    }
}

void SDHDriver::setJointEnabled(int jointIdx, bool enabled){
    try{
        if(enabled)
            _hand->SetAxisEnable(jointIdx, 1.0);
        else
            _hand->SetAxisEnable(jointIdx, 0.0);
    } catch (cSDHLibraryException* e){
        std::string ex = e->what();
        delete e;
        RW_THROW(ex);
    }
}

void SDHDriver::setTimeout(double timeout){
    _moveCmdTimeout = timeout;
}
