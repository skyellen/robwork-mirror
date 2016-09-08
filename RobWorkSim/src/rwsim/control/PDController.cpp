#include "PDController.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;

namespace {
	void setPDParams(const std::vector<PDParam>& pdparams, Q& qp, Q qd){
		qp = Q::zero(pdparams.size());
		qd = Q::zero(pdparams.size());
		for(size_t i=0;i<pdparams.size();i++){
			qp[i] = pdparams[i].P;
			qd[i] = pdparams[i].D;
		}
	}

}

PDController::PDController(
        const std::string& name,
		DynamicDevice::Ptr rdev,
		ControlMode cmode,
		const std::vector<PDParam>& pdparams,
		double dt):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),
	_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(pdparams),
	_mode(cmode),
	_stime(dt),
	_accTime(0),
	_enabled(true)
{
	if(pdparams.size()!=_ddev->getModel().getDOF())
		RW_THROW("Nr of PDParams must match the nr of DOF in the Device!");
	setPDParams(_pdparams,_P,_D);
}

PDController::PDController(
        const std::string& name,
		DynamicDevice::Ptr rdev,
		ControlMode cmode,
		const PDParam& pdparam,
		double dt):
	JointController(name, &rdev->getModel()),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,rdev->getKinematicModel()->getBase()))),
	_ddev(rdev),
	_lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentError(rw::math::Q::zero(rdev->getModel().getDOF())),
	_target(rw::math::Q::zero(rdev->getModel().getDOF())),
	_currentQ(_target),_currentVel(rw::math::Q::zero(_target.size())),
	_targetVel(rw::math::Q::zero(_target.size())),
	_pdparams(rdev->getModel().getDOF(),pdparam),
	_mode(cmode),
	_stime(dt),
	_accTime(0),
	_enabled(true)
{
	setPDParams(_pdparams,_P,_D);
}

void PDController::setTargetPos(const rw::math::Q& target){
    RW_ASSERT_MSG(target.size()==_target.size(), target.size() << "==" << _target.size());
    _target = target;
}

void PDController::setTargetVel(const rw::math::Q& vals){
    RW_ASSERT_MSG(vals.size()==_targetVel.size(), vals.size() << "==" << _targetVel.size());
    _targetVel = vals;
}

void PDController::setTargetAcc(const rw::math::Q& vals){}

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

void PDController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
	if(!_enabled){
		return;
	}

	//_accTime+=dt;
	//if(_accTime<_stime)
	//	return;
    rw::math::Q q = _ddev->getModel().getQ(state);

    if(!info.rollback){
        _lastError = _currentError;
        if(info.dt_prev>0.0){
            _currentVel = (q - _currentQ)/info.dt_prev;
        } else {
            _currentVel = Q::zero(q.size());
        }
    }

	//double rdt = _accTime;
	_accTime -= _stime;


	RW_ASSERT(_target.size()>0);
	RW_ASSERT( q.size()>0);
	_currentError = _target-q;
	rw::math::Q nvel(_currentError.size());

	RW_ASSERT(_pdparams.size()==_lastError.size() );
	RW_ASSERT_MSG(_pdparams.size()==_currentError.size(), _pdparams.size() << "==" << _currentError.size() );

	for(size_t i=0;i<_pdparams.size();i++){
		const double P = _pdparams[i].P;
		const double D = _pdparams[i].D;
		nvel[i] = P*_currentError[i];
		if(info.dt_prev>0.0)
		    nvel[i] += ((_currentError[i]-_lastError[i])/info.dt_prev)*D;
	}

	// std::cout  << "PD ERROR: " << error << std::endl;

	_ddev->setMotorVelocityTargets( (_targetVel + nvel), state);
	//2std::cout  << "T " << _target[0]<< " " << error[0]<< " "<< nvel[0] << " "<< _targetVel[0] << std::endl;
	_currentQ = q;

}

void PDController::reset(const rw::kinematics::State& state){
    _currentQ = _ddev->getModel().getQ(state);
    _target = _currentQ;
    _targetVel = rw::math::Q::zero(_currentQ.size());
    _accTime = 0;
    //_time = 0;
}

void PDController::setControlMode(ControlMode mode){
    if(mode!=POSITION || mode !=VELOCITY )
        RW_THROW("Unsupported control mode!");
    _mode = mode;
}

