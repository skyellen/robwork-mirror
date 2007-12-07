#include "PA10ControlThread.hpp"
#include "PA10.h"
#include "PA10Com.h"


#include <unistd.h>
#include <sys/time.h>
#include <boost/numeric/ublas/vector.hpp>

#include <fstream>
#include <iostream>
using namespace rw::math;
using namespace rw::vworkcell;
using namespace boost::numeric::ublas;

std::ofstream testout;

PA10ControlThread::PA10ControlThread(SerialDevice* device):
    _h(0.01),
    _state(UNINITIALIZED),
    _terminate(false),
    _device(device),
    _jointReal(zero_vector<double>(device->getDOF())),
    _jointTarget(zero_vector<double>(device->getDOF())),
    _jointVel(zero_vector<double>(device->getDOF())),
    _cartTarget(Transform3D<>::Identity()),
    _cartReal(Transform3D<>::Identity()),
    _controller(_h)

 {
     _ramps = new VelRampProfile*[7];
     for (size_t i = 0; i<7; i++) {
	 _ramps[i] = new VelRampProfile(Vector2D<>(PA10::qmin[i], PA10::qmax[i]),
					Vector2D<>(PA10::vmin[i], PA10::vmax[i]),
					Vector2D<>(PA10::amin[i], PA10::amax[i]));
	 
     }
     testout.open("/home/lpe/tmp/testout.dat");

     initPA10();
}


void PA10ControlThread::initPA10() { 
#ifndef ROBOT_DEBUG
    PA10::init_pa10(_jointReal);
#else
    _jointReal = _device->getConfiguration();
#endif
    _device->setConfiguration(_jointReal);
    _jointTarget = _jointReal;
    _jointVel = zero_vector<double>(7);
    _state = SAFE;
}





void PA10ControlThread::run() {
    setPriority(QThread::TimeCriticalPriority);
    //setPriority(QThread::HighPriority);

  //  initPA10();

    timeval startTime, endTime;

    while (!_terminate) {	
	gettimeofday(&startTime, NULL);
	switch (_state) {
	case UNINITIALIZED:
	    std::cerr<<"PA10 Not Initialized Properly"<<std::endl;
	    break;
	case BRAKES_OFF: 
	    brakesOff();
	    _state = READY;
	    break;	   
	case BRAKES_ON:
	  brakesOn();
	    _state = SAFE;
	    break;
	case SAFE:
#ifndef ROBOT_DEBUG	  
	  PA10::lockBrakes();
#endif
	    break;
	case READY:	    
#ifndef ROBOT_DEBUG
	  PA10::freeBrakes();
#endif
	    moveJointPos();
	    break;
	case MOVE_CARTESIAN:
#ifndef ROBOT_DEBUG
	  PA10::freeBrakes();
#endif
	    moveCartesian();
	    break;
	case MOVE_JOINT:
#ifndef ROBOT_DEBUG
	  PA10::freeBrakes();
#endif
	    moveJointPos();
	    break;

	}
	gettimeofday(&endTime, NULL);
	double time = (endTime.tv_sec-startTime.tv_sec)+(endTime.tv_usec-startTime.tv_usec)*1.0e-6;
	if (time < _h) {	  
	    usleep((unsigned long)(1e6*(_h-time)));
	} else {
	  std::cout<<"Warning: Failed to meet time requirement with "<<time-_h<<" seconds "<<std::endl;
	}
    }
}


void PA10ControlThread::pa10BrakesOff() {
  
#ifndef ROBOT_DEBUG
    PA10::freeBrakes();
    _jointTarget = _jointReal;
    Device::Q qdot(zero_vector<double>(7));
    PA10::pa10_send_C(qdot);
    PA10::receive_pos(_jointReal);
#endif
    _jointTarget = _jointReal;

    _device->setConfiguration(_jointReal);    
}


void PA10ControlThread::pa10BrakesOn() {
#ifndef ROBOT_DEBUG
    PA10::lockBrakes();
#endif
}

//#define USEP

void PA10ControlThread::moveJointPos() {
#ifndef ROBOT_DEBUG
    PA10::pa10_send_C(_jointVel);
    PA10::receive_pos(_jointReal);    
#else
    _jointReal += _h*_jointVel;
#endif

#ifndef USEP

    _device->setConfiguration(_jointReal);

  Device::Q vel(7);
    for (size_t i = 0; i<7; i++) {
	vel(i) = _ramps[i]->getVelocity(_jointTarget(i), _jointReal(i), _jointVel(i), _h);
	if (i == 3) {
	    testout<<vel(i)<<" "<<_jointTarget(i)-_jointReal(i)<<std::endl;
	    if (fabs(vel(3)-_jointVel(3))> _h*6+1e-12) {
		char ch[3]; std::cin.getline(ch, 2);
	    }
		
	}
    }
    testout<<std::endl;

    _jointVel = vel;

#else
    _jointVel = (_jointTarget - _jointReal) * 1.0;

#endif



#ifndef ROBOT_DEBUG
    PA10::pa10_send_C(_jointVel);
    PA10::receive_pos(_jointReal);    
#else

#endif


    

}


void PA10ControlThread::moveCartesian() {
#ifndef ROBOT_DEBUG
    PA10::pa10_send_C(_jointVel);      
    PA10::receive_pos(_jointReal);
#else
    timeval startTime, endTime;
    gettimeofday(&startTime, NULL);
#endif
    _device->setConfiguration(_jointReal);


    Transform3D<> wTreal = _device->bTe();    
    Transform3D<> realTdes = inverse(wTreal)*_cartTarget;
    VelocityScrew6D<> vel1(realTdes);
    VelocityScrew6D<> velbase = wTreal*vel1;

    velbase *= 1; //TODO Adjust this parameters to get a quicker/slower response. Works as a simple P controller

    double linvel = norm_2(velbase.linear());
    double angvel = fabs(velbase.angular().angle());

    if (linvel<1e-3 && angvel<1e-3) { //If position error is less than 1mm and rotational error is less than 0.057degrees set both to 0
	velbase *= 0;
    }

    //Safety limits //TODO It might be necessary to adjust these
    if (linvel>2.0 || angvel > M_PI) { //If tool velocity greater than 2m/s or 180deg/s slow it down a bit
	velbase *= std::min(2.0/linvel, M_PI/angvel);
    }
    //TODO If the advanced QPController gives to many problem you can try the standard inverse jacobian implemented in _controller.simpleSolve()
    //_jointVel = _controller.simpleSolve(velbase, _jointTarget, _device);
    _jointVel = _controller.solve(velbase, _jointTarget, _jointVel, _device);
    //_jointTarget += _h*_jointVel;
    //_jointVel = (_jointTarget-_jointReal)/_h;


    // ensure limits
    /*
    if(norm_inf(_jointVel) > M_PI/4.0){
      _jointVel /= norm_inf(_jointVel);
      _jointVel *= M_PI/10.0;
    }
    */






#ifndef ROBOT_DEBUG
    PA10::pa10_send_C(_jointVel);      
    PA10::receive_pos(_jointReal);
#else
    gettimeofday(&endTime, NULL);
    double time = (endTime.tv_sec-startTime.tv_sec)+(endTime.tv_usec-startTime.tv_usec)*1.0e-6;
    if(time < _h)
      _jointReal += (_h - time) * _jointVel;
    //_jointReal += _h*_jointVel; 
    // _jointTarget = _jointReal;
#endif
    //_device->setConfiguration(_jointReal);
}


void PA10ControlThread::moveTo(const Transform3D<>& transform) {
    _cartTarget = transform;
    _state = MOVE_CARTESIAN;
}

void PA10ControlThread::moveTo(const Device::Q& q) {
    _jointTarget = q;
    _state = MOVE_JOINT;
}

void PA10ControlThread::stopMotion() {
    _jointTarget = _jointReal;
    _state = READY;
}

void PA10ControlThread::emergencyStop() {
  
  Transform3D<> t3d(Vector3D<>(-0.36,-0.30,1.19),RPY<>(0.75,0.55,-2.11));
  _cartTarget = t3d;
  _state = MOVE_CARTESIAN;

  /*
#ifndef ROBOT_DEBUG
     PA10::lockBrakes();
#endif
    _terminate = true;
    std::cout<<"Emergency Stop Envoked - Program need to Restart "<<std::endl;
  */
}

void PA10ControlThread::brakesOff() {
    _state = BRAKES_OFF;
}

void PA10ControlThread::brakesOn() {
    _state = BRAKES_ON;
}


void PA10ControlThread::terminate() {
    _terminate = true;
}

