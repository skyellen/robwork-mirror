/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "BodyController.hpp"

#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/RampInterpolator.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwsim::control;
using namespace rwsim::dynamics;

struct BodyController::TargetData {
	typedef enum {Pose6DController, TrajectoryController, VelocityController, ForceController} ControlType;
	TargetData():_type(Pose6DController),_enabled(false){ reset();}
	TargetData(ControlType type):_type(type),_enabled(true){ reset(); }
	void reset(){
		_time = 0;
		_lastTime = 0;
		_lastDt = 0;
	}
	ControlType _type;
	rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr _traj;
	rw::math::VelocityScrew6D<> _velocity;
	rw::math::Transform3D<> _target;
	rw::math::Vector3D<> _force, _torque;
	double _time, // current time on the trajectory
	_lastTime, // the time on the trajectory taken at the last non-rollback step
	_lastDt; // the simulated starting time
	bool _enabled;
};

BodyController::BodyController(const std::string& name):
	Controller(name),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,NULL))),
	_enabled(true)
{
}

BodyController::~BodyController() {
	disableBodyControl();
}

void BodyController::updateKinematicBody(KinematicBody* body, TargetData& tdata, const rwlibs::simulation::Simulator::UpdateInfo& info, State& state){
	// the forcecontroller does not work for kinematic devices
	if(tdata._type==TargetData::ForceController)
		return;
	else if (tdata._type==TargetData::VelocityController) {
		body->setLinVelW(tdata._velocity.linear(), state);
		Vector3D<> angVelW = tdata._velocity.angular().angle()*tdata._velocity.angular().axis();
		body->setAngVelW(angVelW, state);
		if (tdata._velocity.linear().norm2() == 0 && angVelW.norm2() == 0) {
			tdata._enabled = false;

			body->setLinVelW(Vector3D<>::zero(), state);
			body->setAngVelW(Vector3D<>::zero(), state);

			return;
		}
	} else {
		// for a kinematic body we allways follow a trajectory
		// basically we just need the current time to be calculate
		if(!info.rollback){
			// then we need to update the _lastTime
			tdata._lastTime += info.dt_prev;
		}
		double t = tdata._lastTime;

		// test if the trajectory is finished
		if(tdata._traj->duration()<t+info.dt){
			tdata._enabled = false;

			body->setLinVelW(Vector3D<>(0, 0, 0) , state);
			body->setAngVelW(Vector3D<>(0, 0, 0), state);

			return;
		}

		// the target position for next step.
		Transform3D<> nextPose = tdata._traj->x(t+info.dt);

		// set angular and linear velocities of body such that it will move toward nextPose in time info.dt

		Transform3D<> wTt = nextPose;
		Transform3D<> wTb = Kinematics::worldTframe(body->getBodyFrame(), state);

		const Transform3D<>& bTt = inverse(wTb) * wTt; // eTed
		const VelocityScrew6D<> vel( bTt );
		const VelocityScrew6D<> velW = (wTb.R() * vel);

		for (std::size_t i = 0; i < 6; i++) {
			if(Math::isNaN(vel[i])) {
				RW_THROW("BodyController encountered nan value for velocity screw of Kinematic body " << body->getName() << ".");
			}
		}

		Vector3D<> vErrW = velW.linear()/info.dt;
		// check if the current velocity will overshoot the target if we have a constant deacceleration of ACC_MAX
		// compute arrive time assuming konstant deacceleration
		body->setLinVelW( vErrW , state);

		// and now control the angular velocity
		Vector3D<> angVelW(velW(3),velW(4),velW(5));

		body->setAngVelW(angVelW/info.dt, state);
	}
}

void BodyController::updateBody(Body* body, TargetData& tdata,
		const rwlibs::simulation::Simulator::UpdateInfo& info,
		State& state)
{
	// for a kinematic body we allways follow a trajectory
	// basically we just need the current time to be calculate
	if(!info.rollback){
		// then we need to update the _lastTime
		tdata._lastTime += info.dt_prev;
	}
	double t = tdata._lastTime;

	if(tdata._type==TargetData::ForceController){
		body->setForceW( info.dt*tdata._force, state);
		body->setTorqueW( info.dt*tdata._torque, state);
	} else if (tdata._type==TargetData::VelocityController) {
		RW_THROW("Velocity targets not supported yet for non-kinematic bodies!");
	} else {

		// test if the trajectory is finished
		if(tdata._traj->duration()<t+info.dt){
			tdata._enabled = false;
			return;
		}

		// the target position for next step.
		const Transform3D<> nextPose = tdata._traj->x(t+info.dt);

		// TODO: apply a wrench to the body such that is will move toward nextPose in time info.dt
		// for now we just create simple penalty based on the velocity
		const Transform3D<> wTt = nextPose;
		const Transform3D<> wTb = Kinematics::worldTframe(body->getBodyFrame(), state);

		const Transform3D<>& bTt = inverse(wTb) * wTt; // eTed
		const VelocityScrew6D<> vel( bTt );
		const VelocityScrew6D<> velW = (wTb.R() * vel);

		const Vector3D<> vErrW = velW.linear()/info.dt;
		// and now control the angular velocity
		const Vector3D<> aErrW = Vector3D<>(velW(3),velW(4),velW(5))/info.dt;

		// get the current velocities
		const Vector3D<> vCurW = body->getLinVelW(state);
		const Vector3D<> aCurW = body->getAngVelW(state);

		const double mass = body->getInfo().mass;
		const InertiaMatrix<> inertia = body->getInfo().inertia;
		const InertiaMatrix<> inertiaW = wTb.R()*inertia*inverse(wTb.R());

		body->setForceW( (vErrW-vCurW)*mass*10 , state);
		body->setTorqueW( inertiaW*(aErrW-aCurW)*10 , state);

	}
}

void BodyController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
	boost::mutex::scoped_lock lock(_mutex);

    //std::cout << "B" << std::endl;
    //Log::infoLog() << "update";
	std::map<Body*, TargetData*>::iterator it;
	for (it = _bodyMap.begin(); it != _bodyMap.end(); it++) {
        BodyController::TargetData* tdata = it->second;
        if(!tdata->_enabled)
            continue;
        if( KinematicBody *kbody = dynamic_cast<KinematicBody*>(it->first) ){
            updateKinematicBody(kbody, *tdata, info, state );
        } else {
            updateBody(it->first, *tdata, info, state );
        }
    }
}

void BodyController::disableBodyControl(Body::Ptr body) {
	boost::mutex::scoped_lock lock(_mutex);

    if(_bodyMap.find(body.get())==_bodyMap.end())
        return;

    _bodyMap[body.get()]->_enabled = false;
}

void BodyController::disableBodyControl(){
	boost::mutex::scoped_lock lock(_mutex);
	std::map<Body*, TargetData*>::iterator it;
	for (it = _bodyMap.begin(); it != _bodyMap.end(); it++) {
		delete it->second;
	}
    _bodyMap.clear();
}

void BodyController::reset(const State& state) {
}

void BodyController::setTarget(Body::Ptr body, const Transform3D<>& target, const State& state, double maxLinVel, double maxLinAcc, double maxAngVel, double maxAngAcc) {
	boost::mutex::scoped_lock lock(_mutex);

    // create a trajectory using a velocity ramp function
    Transform3D<> from = Kinematics::worldTframe(body->getBodyFrame(), state);
    if(MetricUtil::dist2(from.P(),target.P())<0.00001){
        return;
    }

    if(_bodyMap.find(body.get())==_bodyMap.end())
    	_bodyMap[body.get()] = new TargetData();

    TargetData& data = *_bodyMap[body.get()];
    data.reset();
    data._type = TargetData::Pose6DController;

    RampInterpolator<Transform3D<> >::Ptr ramp =
            rw::common::ownedPtr( new RampInterpolator<Transform3D<> >( from , target, maxLinVel, maxLinAcc, maxAngVel, maxAngAcc));
    InterpolatorTrajectory<Transform3D<> >::Ptr traj =
            rw::common::ownedPtr( new InterpolatorTrajectory<Transform3D<> >() );
    traj->add(ramp);
    //Log::infoLog() << "from  : " << from << " \n";
    //Log::infoLog() << "target: " << target << " \n";
    //Log::infoLog() << "ramp duration: " << ramp->duration() << " \n";
    //Log::infoLog() << "traj duration: " << traj->duration() << " \n";
    data._traj = traj;
    data._target = target;
    data._enabled = true;

}

void BodyController::setTarget(Body::Ptr body, Trajectory<Transform3D<> >::Ptr traj) {
	boost::mutex::scoped_lock lock(_mutex);

    if(_bodyMap.find(body.get())==_bodyMap.end())
    	_bodyMap[body.get()] = new TargetData();

    TargetData& data = *_bodyMap[body.get()];
    data.reset();
    data._type = TargetData::TrajectoryController;
    data._traj = traj;
    data._target = traj->x( traj->endTime() );
    data._enabled = true;
}

void BodyController::setTarget(Body::Ptr body, const VelocityScrew6D<> &velocity) {
	boost::mutex::scoped_lock lock(_mutex);

    if(_bodyMap.find(body.get())==_bodyMap.end())
    	_bodyMap[body.get()] = new TargetData();

    TargetData& data = *_bodyMap[body.get()];
    data.reset();
    data._type = TargetData::VelocityController;
    data._velocity = velocity;
	data._enabled = true;
}

void BodyController::setForceTarget(Body::Ptr body,
                    Vector3D<> force,
                    Vector3D<> torque)
{
	boost::mutex::scoped_lock lock(_mutex);

    if(_bodyMap.find(body.get())==_bodyMap.end())
    	_bodyMap[body.get()] = new TargetData();

    TargetData& data = *_bodyMap[body.get()];
    data.reset();
    data._type = TargetData::ForceController;
    data._force = force;
    data._torque = torque;
    data._enabled = true;
}


rw::math::Transform3D<> BodyController::getTarget(Body::Ptr body) {
	boost::mutex::scoped_lock lock(_mutex);
    if( _bodyMap.find(body.get())!=_bodyMap.end() ){
        return _bodyMap[body.get()]->_target;
    }
    return Transform3D<>::identity();
}

Trajectory<Transform3D<> >::Ptr BodyController::getTargetTrajectory(Body::Ptr body) {
	boost::mutex::scoped_lock lock(_mutex);
    if(_bodyMap.find(body.get())==_bodyMap.end())
    	_bodyMap[body.get()] = new TargetData();
    return _bodyMap[body.get()]->_traj;
}
