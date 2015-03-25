#include "BeamJointController.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;

namespace {

}

BeamJointController::BeamJointController(
        const std::string& name,
        RigidDevice* rdev,
		ControlMode cmode,
		double dt):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),
	_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_mode(cmode),
	_stime(dt)
{

}

void BeamJointController::setTargetPos(const rw::math::Q& target){
    _target = target;
}

void BeamJointController::setTargetVel(const rw::math::Q& vals){
	_targetVel = vals;
}

void BeamJointController::setTargetAcc(const rw::math::Q& vals){};


double BeamJointController::getSampleTime(){
	return _stime;
}

void BeamJointController::setSampleTime(double stime){
	_stime = stime;
}

void BeamJointController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    // all joints in the device are dependent on a single input
    double pressure = _target[0];

    double angle = pressure;

    // the pressure indicate the size of the torques that are applied on the beam joints.
    // the closer the beamjoints are at thier resting configuration the smaller torque
	Q q = _ddev->getModel().getQ(state);
	Q q_error = Q(q.size(), angle)-q;
	// the error in configuration result in a torque
	Q torque = q;

	for(size_t i=0;i<q.size();i++){ torque[i] = 2; }

	_ddev->setMotorForceLimits(torque);
	_ddev->setMotorVelocityTargets(q_error, state);
	_currentQ = q;
}

void BeamJointController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    //_time = 0;
}

void BeamJointController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

