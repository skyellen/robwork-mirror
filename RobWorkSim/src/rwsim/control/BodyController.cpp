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
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/RampInterpolator.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rw::common;


BodyController::BodyController(const std::string& name):
	Controller(name),
	SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,NULL)))
{
}

namespace {
    const double MAX_LIN_ACCELERATION = 3; // m/s^2
    const double MAX_LIN_VELOCITY = 0.5; // m/s


    void updateKinematicBody(KinematicBody* body, BodyController::TargetData& tdata, const rwlibs::simulation::Simulator::UpdateInfo& info, State& state){
        // the forcecontroller does not work for kinematic devices
        if(tdata._type==BodyController::ForceController)
            return;
        else if (tdata._type==BodyController::VelocityController) {
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

    void updateBody(Body* body, BodyController::TargetData& tdata,
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

        if(tdata._type==BodyController::ForceController){
            body->setForceW( info.dt*tdata._force, state);
            body->setTorqueW( info.dt*tdata._torque, state);
        } else if (tdata._type==BodyController::VelocityController) {
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
}

void BodyController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
	boost::mutex::scoped_lock lock(_mutex);

    //std::cout << "B" << std::endl;
    //Log::infoLog() << "update";
    BOOST_FOREACH(Body::Ptr body, _bodies){
        BodyController::TargetData& tdata = _bodyMap[body.get()];
        if(!tdata._enabled)
            continue;
        if( KinematicBody *kbody = dynamic_cast<KinematicBody*>(body.get()) ){
            updateKinematicBody(kbody, tdata, info, state );
        } else {
            updateBody(body.get(), tdata, info, state );
        }

    }

}

void BodyController::disableBodyControl(rwsim::dynamics::Body::Ptr body){
	boost::mutex::scoped_lock lock(_mutex);

    if(_bodyMap.find(body.get())==_bodyMap.end())
        return;

    _bodyMap[body.get()]._enabled = false;
}

void BodyController::disableBodyControl(){
	boost::mutex::scoped_lock lock(_mutex);

    _bodyMap.clear();
    _bodies.clear();
}

void BodyController::reset(const rw::kinematics::State& state){

    //_time = 0;
}

void BodyController::setTarget(rwsim::dynamics::Body::Ptr body,
                               const rw::math::Transform3D<>& target,
                               rw::kinematics::State& state)
{
	boost::mutex::scoped_lock lock(_mutex);

    // create a trajectory using a velocity ramp function
    Transform3D<> from = Kinematics::worldTframe(body->getBodyFrame(), state);
    if(MetricUtil::dist2(from.P(),target.P())<0.00001){
        return;
    }

    TargetData& data = _bodyMap[body.get()];
    data.reset();
    data._type = Pose6DController;

    RampInterpolator<Transform3D<> >::Ptr ramp =
            rw::common::ownedPtr( new RampInterpolator<Transform3D<> >( from , target, 0.5, 1, 0.4, 1));
    InterpolatorTrajectory<Transform3D<> >::Ptr traj =
            rw::common::ownedPtr( new InterpolatorTrajectory<Transform3D<> >() );
    traj->add(ramp);
    //Log::infoLog() << "from  : " << from << " \n";
    //Log::infoLog() << "target: " << target << " \n";
    //Log::infoLog() << "ramp duration: " << ramp->duration() << " \n";
    //Log::infoLog() << "traj duration: " << traj->duration() << " \n";
    data._traj = traj;
    data._target = target;
    _bodyMap[body.get()]._enabled = true;
    _bodies.push_back(body);

}

void BodyController::setTarget(Body::Ptr body, rw::trajectory::Trajectory<Transform3D<> >::Ptr traj, State& state){
	boost::mutex::scoped_lock lock(_mutex);

    TargetData& data = _bodyMap[body.get()];
    data.reset();
    data._type = TrajectoryController;
    data._traj = traj;
    data._target = traj->x( traj->endTime() );
    _bodyMap[body.get()]._enabled = true;
    _bodies.push_back(body);
}

void BodyController::setTarget(Body::Ptr body, const VelocityScrew6D<> &velocity, State& state) {
	boost::mutex::scoped_lock lock(_mutex);

    TargetData& data = _bodyMap[body.get()];
    data.reset();
    data._type = VelocityController;
    data._velocity = velocity;
    _bodyMap[body.get()]._enabled = true;
    _bodies.push_back(body);
}

void BodyController::setForceTarget(rwsim::dynamics::Body::Ptr body,
                    rw::math::Vector3D<> force,
                    rw::math::Vector3D<> torque,
                    rw::kinematics::State& state)
{
	boost::mutex::scoped_lock lock(_mutex);

    TargetData& data = _bodyMap[body.get()];
    data.reset();
    data._type = ForceController;
    data._force = force;
    data._torque = torque;
    _bodyMap[body.get()]._enabled = true;
    _bodies.push_back(body);
}


rw::math::Transform3D<> BodyController::getTarget(rwsim::dynamics::Body::Ptr body){
	boost::mutex::scoped_lock lock(_mutex);

    if( _bodyMap.find(body.get())!=_bodyMap.end() ){
        // add body to control
        return _bodyMap[body.get()]._target;
    }
    return Transform3D<>::identity();
}

Trajectory<Transform3D<> >::Ptr BodyController::getTargetTrajectory(Body::Ptr body){
	boost::mutex::scoped_lock lock(_mutex);

    return _bodyMap[body.get()]._traj;
}
