#include "SpringJointController.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;

namespace {

}

SpringJointController::SpringJointController(
        const std::string& name,
        RigidDevice::Ptr rdev,
		const std::vector<SpringParam>& springParam,
		double dt):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),
	_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_stime(dt),
	_springParams(springParam)
{

}

void SpringJointController::setTargetPos(const rw::math::Q& target){
    _target = target;
}

void SpringJointController::setTargetVel(const rw::math::Q& vals){
	_targetVel = vals;
}

void SpringJointController::setTargetAcc(const rw::math::Q& vals){};


double SpringJointController::getSampleTime(){
	return _stime;
}

void SpringJointController::setSampleTime(double stime){
	_stime = stime;
}

void SpringJointController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    // all joints in the device are dependent on a single input

    // the pressure indicate the size of the torques that are applied on the beam joints.
    // the closer the beamjoints are at their resting configuration the smaller torque

    Q q = _ddev->getModel().getQ(state);
    Q q_error = _currentQ-q;
    if(info.rollback){
        // then we use the last calculated error
        q_error = _qError;
    } else {
        _qError = q_error;
    }


    // the error in configuration result in a torque
    Q torque(q_error.size());
    if(info.dt_prev>0.0){
        for(size_t i=0; i<torque.size(); i++)
            torque(i) = (q(i)+_springParams[i].offset)*_springParams[i].elasticity - (q_error(i)*_springParams[i].dampening)/info.dt_prev;
    } else {
        for(size_t i=0; i<torque.size(); i++)
            torque(i) = (q(i)+_springParams[i].offset)*_springParams[i].elasticity;
    }

    //std::cout << "ErrorRoll: " << info.time << ", " << q_error << std::endl;
    //std::cout << torque << std::endl;
    //for(int i=0;i<q_error.size();i++){
    //    std::cout << q_error(i) << "; ";
    //}
    //for(int i=0;i<torque.size();i++){
    //    std::cout << torque(i) << "; ";
    //}
    //std::cout << std::endl;

    _ddev->setMotorForceTargets(torque, state);


    //_ddev->setForceLimit(torque);
    //_ddev->setVelocity(q_error, state);

    _currentQ = q;

}

void SpringJointController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    //_time = 0;
}

void SpringJointController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

