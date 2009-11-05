/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "SDHDriver.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/common/TimerUtil.hpp>

#include <sdh/sdh.h>
#include <rwhw/can/ESDCAN/ESDCANPort.hpp>

#include <iostream>
#include <float.h>


using namespace SDH;

using namespace rw::math;
using namespace rwhw;

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

SDHDriver::SDHDriver(){
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

}

bool SDHDriver::connect( ESDCANPort *cport ){
	//HANDLE handle = cport->getHandle();
	unsigned int net = 0;
	unsigned long baud = 1000000;
	double timeout = 1.0;

	try {
	    _hand->OpenCAN_ESD( net, baud , timeout, 43, 42);
	} catch (cSDHLibraryException* e){
	    Log::error() << e.what();
	    delete e;
	    return false;
	}

	try {
        _hand->SetVelocityProfile( cSDH::eVP_RAMP );
        _min = toQ(_hand->GetAxisMinAngle(_axes));
        _max = toQ(_hand->GetAxisMaxAngle(_axes));
	} catch (cSDHLibraryException* e){
        Log::error() << e.what();
        _hand->Close();
        delete e;
        return false;
    }

	//std::cout << "Min: " << _min << std::endl;
	//std::cout << "Max: " << _max << std::endl;

	// Open communication to the SDH device via default serial port 0 == "COM1"
	//hand.OpenRS232( options.port, options.rs232_baudrate, options.timeout );
	return isConnected();
}

bool SDHDriver::isConnected(){
	return _hand->IsOpen();
}

void SDHDriver::disconnect(){
	_hand->Close();
}

void SDHDriver::moveCmd(rw::math::Q target, bool block){
	// adjust the dependent joint
	rw::math::Q tq = Math::clampQ(target,_min,_max);
	setStdVector(_vjointTmp, tq);
	try {
		_hand->SetAxisTargetAngle(_axes, _vjointTmp);
		_hand->MoveAxis(_axes, block);
	} catch (cSDHLibraryException* e){
		std::cout << e->what() << std::endl;
		delete e;
	}
}

void SDHDriver::moveJointCmd(int jointIdx, double target, bool block){
	try {
		_hand->SetAxisTargetAngle(jointIdx, target);
		_hand->MoveAxis(_axes, block);
	} catch (cSDHLibraryException* e){
		std::cout << e->what() << std::endl;
		delete e;
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
	std::vector<double> target = _hand->GetAxisTargetAngle(_axes);
	return toQ(target);
}

void SDHDriver::setTargetQVel(const rw::math::Q& jointVel){
	setStdVector(_vjointTmp, jointVel);
	_hand->SetAxisTargetVelocity(_axes, _vjointTmp );
}

void SDHDriver::setTargetQAcc(const rw::math::Q& jointAcc){
	setStdVector(_vjointTmp, jointAcc);
	_hand->SetAxisTargetAcceleration(_axes, _vjointTmp );
}

void SDHDriver::setTargetQCurrent(const rw::math::Q& jointCurr){
	setStdVector(_vjointTmp, jointCurr);
	_hand->SetAxisMotorCurrent(_axes, _vjointTmp );
}

rw::math::Q SDHDriver::getQ(){
	std::vector<double> target;
	try{
		 target = _hand->GetAxisActualAngle(_axes);
	} catch (cSDHLibraryException* e){
		std::cout << e->what() << std::endl;
		delete e;
		return Q::zero(7);
	}
	return toQ(target);
}

rw::math::Q SDHDriver::getdQ(){
	std::vector<double> vel = _hand->GetAxisActualVelocity(_axes);
	return toQ(vel);
}

void SDHDriver::stop(){
	_hand->Stop();
}

int SDHDriver::getDOF(){
	return _hand->GetNumberOfAxes();
};

void SDHDriver::emergencyStop(){
	_hand->EmergencyStop();
}

std::pair<rw::math::Q,rw::math::Q> SDHDriver::getPosLimits(){
	Q minQ = toQ(_hand->GetAxisMinAngle(_axes));
	Q maxQ = toQ(_hand->GetAxisMaxAngle(_axes));
	return std::pair<Q,Q>(minQ,maxQ);
}

rw::math::Q SDHDriver::getVelLimits(){
	std::vector<double> vellimits = _hand->GetAxisMaxVelocity(_axes);
	return toQ(vellimits);
}

rw::math::Q SDHDriver::getAccLimits(){
	std::vector<double> acclimits = _hand->GetAxisMaxAcceleration(_axes);
	return toQ(acclimits);
}

rw::math::Q SDHDriver::getCurrentLimits(){
	std::vector<double> current =
		_hand->GetAxisMaxMotorCurrent(_axes );
	return toQ(current);
}

void SDHDriver::setJointEnabled(bool enabled){

}

void SDHDriver::setJointEnabled(int jointIdx, bool enabled){

}

void SDHDriver::setTimeout(int timeout){

}
