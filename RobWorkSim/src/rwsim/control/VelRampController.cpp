#include "VelRampController.hpp"

using namespace rwsim::control;

VelRampController::VelRampController(const std::string& name, rwsim::dynamics::KinematicDevice* kdev, const rw::kinematics::State& state):
	JointController(name, &kdev->getModel()),
	SimulatedController( rw::common::ownedPtr(new rw::models::ControllerModel(name,kdev->getModel().getBase())) ),
	_ddev(kdev),
	_time(0.0),
	_velramp(&(kdev->getModel())),
	_target(kdev->getModel().getQ(state)),
	_currentQ(_target)
{
	_velramp.setTarget(_target,_target);

}


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
