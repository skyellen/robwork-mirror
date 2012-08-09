#include "VelRampController.hpp"

using namespace rwsim::control;

void VelRampController::setTargetPos(const rw::math::Q& target){
    rw::math::Q q = _velramp.x(_time);
    _velramp.setTarget(q,target);
    _time = 0;
    _target = target;
}

void VelRampController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    _time += info.dt;
    rw::math::Q qvel = _velramp.dx(_time);

    std::cout << "Setting Qvelocity: "<< qvel << std::endl;
    _ddev->setMotorVelocityTargets(qvel, state);
    _currentQ = qvel;
}

void VelRampController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _velramp.setTarget(_currentQ, _target);
    _time = 0;
}

rw::math::Q VelRampController::getQ(){
    return _target;
}
