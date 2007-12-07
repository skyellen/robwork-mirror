#ifndef PA10CONTROLTHREAD_HPP
#define PA10CONTROLTHREAD_HPP

//TODO ROBOT_DEBUG means only simulate the robot. 
#define ROBOT_DEBUG 

#include <QThread>
#include <math/Transform3D.hpp>
#include <vworkcell/Device.hpp>
#include <vworkcell/SerialDevice.hpp>

#include "QPController.hpp"
#include "VelRampProfile.hpp"

class PA10ControlThread: public QThread {
public:
    PA10ControlThread(rw::vworkcell::SerialDevice* device);


    void run();


    void brakesOff();
    void brakesOn();

    void moveTo(const rw::math::Transform3D<>& transform);
    
    void moveTo(const rw::vworkcell::Device::Q& q);

    void stopMotion();

    void emergencyStop();
	
    
    
    void terminate();



private:

    void initPA10();

    void pa10BrakesOff();

    void pa10BrakesOn();

    void moveJointPos();

    void moveCartesian();



    enum State { UNINITIALIZED = 0, BRAKES_OFF, BRAKES_ON, SAFE, READY, MOVE_CARTESIAN, MOVE_JOINT };
    double _h;
    State _state;
    bool _terminate;

    rw::vworkcell::SerialDevice* _device;
    rw::vworkcell::Device::Q _jointReal;
    rw::vworkcell::Device::Q _jointTarget;
    rw::vworkcell::Device::Q _jointVel;

    rw::math::Transform3D<> _cartTarget;
    rw::math::Transform3D<> _cartReal;

    QPController _controller;
    VelRampProfile** _ramps;
};

#endif //#ifndef PA10CONTROLTHREAD_HPP
