#include "SyncPDController.hpp"

#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rwsim::control;

SyncPDController::SyncPDController(const std::string& name, rwsim::dynamics::RigidDevice* rdev, const rw::kinematics::State& state):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_time(0.0),
	_target(rdev->getModel().getQ(state)),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_velramp(&(rdev->getModel())),
	_currentQ(_target)
{
	_velramp.setTarget(_target,_target);
}


void SyncPDController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

void SyncPDController::setTargetPos(const rw::math::Q& target){
    _velramp.setTarget(_currentQ,target);
    _target = target;
}

void SyncPDController::setTargetVel(const rw::math::Q& vals){
	_targetVel = vals;
}

void SyncPDController::setTargetAcc(const rw::math::Q& vals){}

/**
 * @brief updates the state of the dynamicdevice
 */
void SyncPDController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    const double P = 4;
    const double D = 0.3;
    _currentQ = _ddev->getModel().getQ(state);
    _time += info.time;

    // use both position error to compensate for velocity error
    rw::math::Q q = _velramp.x(_time);
    rw::math::Q dq = (q-_x)/info.dt;
    //rw::math::Q qd = _velramp.xd(_time);

    rw::math::Q error = q-_currentQ;
    rw::math::Q nvel = P*error + (error-_lastError)*D;
    _lastError = error;
    _ddev->setMotorVelocityTargets(nvel + dq, state);
    _x = q;

    std::cout  << "------ Debug SyncPDController: " << std::endl;
    std::cout  << "- Q: " << q << std::endl;
    std::cout  << "- Qd: " << dq << std::endl;
    std::cout  << "- NVel: " << nvel << std::endl;
    std::cout  << "- Error: " << error << std::endl;
}

void SyncPDController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _x = _currentQ;
    _velramp.setTarget(_currentQ, _target);
    _time = 0;

}
