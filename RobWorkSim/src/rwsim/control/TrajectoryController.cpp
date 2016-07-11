#include "TrajectoryController.hpp"

#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rwsim::control;

void TrajectoryController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

void TrajectoryController::setTargetPos(const rw::math::Q& target){
    _velramp.setTarget(_currentQ,target);
    _target = target;
}

void TrajectoryController::setTargetVel(const rw::math::Q& vals){};

void TrajectoryController::setTargetAcc(const rw::math::Q& vals){};

void TrajectoryController::update(double dt, rw::kinematics::State& state) {
    const double P = 4;
    const double D = 0.3;
    _currentQ = _ddev->getModel().getQ(state);
    _time += dt;

    // use both position error to compensate for velocity error
    rw::math::Q q = _velramp.x(_time);
    rw::math::Q dq = (q-_x)/dt;
    //rw::math::Q qd = _velramp.xd(_time);

    rw::math::Q error = q-_currentQ;
    rw::math::Q nvel = P*error + (error-_lastError)*D;
    _lastError = error;
    _ddev->setMotorVelocityTargets(nvel + dq, state);
    _x = q;

    std::cout  << "------ Debug TrajectoryController: " << std::endl;
    std::cout  << "- Q: " << q << std::endl;
    std::cout  << "- Qd: " << dq << std::endl;
    std::cout  << "- NVel: " << nvel << std::endl;
    std::cout  << "- Error: " << error << std::endl;
}

void TrajectoryController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _x = _currentQ;
    _velramp.setTarget(_currentQ, _target);
    _time = 0;

}
