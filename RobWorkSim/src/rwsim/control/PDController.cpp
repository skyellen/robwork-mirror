#include "PDController.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;

namespace {
	void setPDParams(const std::vector<PDParam>& pdparams, Q& qp, Q qd){
		qp = Q::zero(pdparams.size());
		qd = Q::zero(pdparams.size());
		for(int i=0;i<pdparams.size();i++){
			qp[i] = pdparams[i].P;
			qd[i] = pdparams[i].D;
		}
	}

}

PDController::PDController(
		RigidDevice* rdev, const rw::kinematics::State& state,
		ControlMode cmode,
		const std::vector<PDParam>& pdparams,
		double dt):
	JointController(&rdev->getModel()),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rdev->getModel().getQ(state)),
	_currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(pdparams),
	_mode(cmode),
	_stime(dt)
{
	if(pdparams.size()!=_ddev->getModel().getDOF())
		RW_THROW("Nr of PDParams must match the nr of DOF in the Device!");
	setPDParams(_pdparams,_P,_D);
}

PDController::PDController(
		RigidDevice* rdev, const rw::kinematics::State& state,
		ControlMode cmode,
		const PDParam& pdparam,
		double dt):
	JointController(&rdev->getModel()),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rdev->getModel().getQ(state)),
	_currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(rdev->getModel().getDOF(),pdparam),
	_mode(cmode),
	_stime(dt)
{
	setPDParams(_pdparams,_P,_D);
}

void PDController::setTargetPos(const rw::math::Q& target){
    _target = target;
}

void PDController::setTargetVel(const rw::math::Q& vals){
	_targetVel = vals;
}

void PDController::setTargetAcc(const rw::math::Q& vals){};

std::vector<PDParam> PDController::getParameters(){
	return _pdparams;
}

void PDController::setParameters(const std::vector<PDParam>& params){
	if(params.size()!=_ddev->getModel().getDOF())
		RW_THROW("Nr of PDParams must match the nr of DOF in the Device!");
	_pdparams = params;
	setPDParams(_pdparams,_P,_D);
}

double PDController::getSampleTime(){
	return _stime;
}

void PDController::setSampleTime(double stime){
	_stime = stime;
}

void PDController::update(double dt, rw::kinematics::State& state) {
	_accTime+=dt;
	//std::cout << "1" << std::endl;
	if(_accTime<_stime)
		return;

	double rdt = _accTime;
	_accTime -= _stime;
	rw::math::Q q = _ddev->getModel().getQ(state);
	rw::math::Q error = _target-q;
	rw::math::Q nvel(error.size());
	RW_ASSERT(_pdparams.size()==_lastError.size() );
	RW_ASSERT(_pdparams.size()==error.size() );

	for(size_t i=0;i<_pdparams.size();i++){
		const double P = _pdparams[i].P;
		const double D = _pdparams[i].D;
		nvel[i] = P*error[i] + ((error[i]-_lastError[i])/rdt)*D;
	}
	// std::cout  << "PD TARGET: " << _target << std::endl;
	// std::cout  << "PD ERROR: " << error << std::endl;

	_lastError = error;
	_ddev->setVelocity(_targetVel + nvel, state);
	_currentVel = (q - _currentQ)/rdt;
	_currentQ = q;

}

void PDController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    //_time = 0;
}

void PDController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

