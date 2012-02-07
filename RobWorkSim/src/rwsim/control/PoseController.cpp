#include "PoseController.hpp"

#include <rw/common/macros.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rwlibs::algorithms;

PoseController::PoseController(
        const std::string& name,
		DynamicDevice* rdev,
		const rw::kinematics::State& state,
		double dt):
	Controller(name),
	_ddev(rdev),
    _device( _ddev->getKinematicModel() ),
    _endframe( _device->getEnd() ),
	_target(_device->baseTframe(_endframe, state)),
	_targetVel(0,0,0,0,0,0),
	_stime(dt),
	_accTime(0),
	_xqp( rw::common::ownedPtr(new XQPController(_device, _endframe, state, _stime)))
{
}

PoseController::PoseController(
        const std::string& name,
		DynamicDevice* rdev,
		const rw::kinematics::State& state,
		double dt,
		rw::kinematics::Frame* endframe):
	Controller(name),
    _ddev(rdev),
    _device( _ddev->getKinematicModel() ),
    _endframe( endframe ),
    _target(_device->baseTframe(_endframe, state)),
    _targetVel(0,0,0,0,0,0),
    _stime(dt),
    _accTime(0),
	_xqp( ownedPtr(new XQPController(_device, _endframe, state, _stime)))
{
}

void PoseController::setTarget(const rw::math::Transform3D<>& target){
    // position control mode
    _target = target;
    _targetVel = VelocityScrew6D<>(0,0,0,0,0,0);
}

void PoseController::setTarget(const rw::math::Transform3D<>& target, const rw::math::VelocityScrew6D<>& vals){
    // velocity control mode
    _target = target;
    _targetVel = vals;
}

double PoseController::getSampleTime(){
	return _stime;
}

void PoseController::setSampleTime(double stime){
	_stime = stime;
}

void PoseController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    // there might be two situations, rollback or not.

    //_device->setQ(_q, _state);
    Frame* tcpFrame = _endframe;
    Transform3D<> Tcurrent = _device->baseTframe(tcpFrame, state);
    Transform3D<> Tdiff = inverse(Tcurrent)*_target;
    Q q = _device->getQ(state);
    Q dq = _ddev->getVelocity(state);

    VelocityScrew6D<> vs(Tdiff);

    // we use a small gain
    double gain = 0.1;
    VelocityScrew6D<> diff = gain*(Tcurrent.R()*vs);
    diff += _targetVel;

    double linvel = diff.linear().norm2();

    const double maxLinearVelocity = 0.5;
    if (linvel > maxLinearVelocity) {
        diff *= maxLinearVelocity/linvel;
    }

    const double maxAngularVelocity = 0.5;
    if (diff.angular().angle() > maxAngularVelocity) {
        diff *= maxAngularVelocity/diff.angular().angle();
    }

    //log(Info)<<"Call PseudoInverse"<<endlog();
    Q dqtarget = _xqp->solve(q, dq, diff, std::list<XQPController::Constraint>());
    dq = dqtarget;
    //q += _dt*_dq;

    _ddev->setVelocity( dq, state);

}

void PoseController::reset(const rw::kinematics::State& state){
    Frame* tcpFrame = _endframe;
    Transform3D<> Tcurrent = _device->baseTframe(tcpFrame, state);
    _xqp = new XQPController(_device, _endframe, state, _stime);
    _target = Tcurrent;
    _targetVel = VelocityScrew6D<>(0,0,0,0,0,0);
    _accTime = 0;
    //_time = 0;
}

