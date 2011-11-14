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
	Controller(name)
{
}

namespace {
    const double MAX_LIN_ACCELERATION = 3; // m/s^2
    const double MAX_LIN_VELOCITY = 0.5; // m/s


    void updateKinematicBody(KinematicBody* body, BodyController::TargetData& tdata, const rwlibs::simulation::Simulator::UpdateInfo& info, State& state){
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

        Vector3D<> vErrW = velW.linear()/info.dt;
        // check if the current velocity will overshoot the target if we have a constant deacceleration of ACC_MAX
        // compute arrive time assuming konstant deacceleration
        body->setLinVelW( vErrW , state);

        // and now control the angular velocity
        Vector3D<> angVelW(velW(3),velW(4),velW(5));

        body->setAngVelW(angVelW/info.dt, state);
    }

    void updateBody(Body* body, BodyController::TargetData& tdata, const rwlibs::simulation::Simulator::UpdateInfo& info, State& state){

    }
}

void BodyController::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state) {
    //std::cout << "B" << std::endl;
    //Log::infoLog() << "update";
    BOOST_FOREACH(Body* body, _bodies){
        BodyController::TargetData& tdata = _bodyMap[body];
        if(!tdata._enabled)
            continue;
        if( KinematicBody *kbody = dynamic_cast<KinematicBody*>(body) ){
            updateKinematicBody(kbody, tdata, info, state );
        } else {

            //updateBody(rbody, tdata, info, state );

        }

    }

}

void BodyController::disableBodyControl(rwsim::dynamics::Body *body){
    if(_bodyMap.find(body)==_bodyMap.end())
        return;

    _bodyMap[body]._enabled = false;
}

void BodyController::disableBodyControl(){
    _bodyMap.clear();
    _bodies.clear();
}

void BodyController::reset(const rw::kinematics::State& state){

    //_time = 0;
}

void BodyController::setTarget(rwsim::dynamics::Body *body,
                               const rw::math::Transform3D<>& target,
                               rw::kinematics::State& state)
{

    // create a trajectory using a velocity ramp function
    Transform3D<> from = Kinematics::worldTframe(body->getBodyFrame(), state);
    if(MetricUtil::dist2(from.P(),target.P())<0.00001){
        return;
    }

    TargetData& data = _bodyMap[body];
    data.reset();
    data._type = Pose6DController;

    RampInterpolator<Transform3D<> >::Ptr ramp =
            rw::common::ownedPtr( new RampInterpolator<Transform3D<> >( from , target, 0.5, 0.03, 0.4, 0.1));
    InterpolatorTrajectory<Transform3D<> >::Ptr traj =
            rw::common::ownedPtr( new InterpolatorTrajectory<Transform3D<> >() );
    traj->add(ramp);
    //Log::infoLog() << "from  : " << from << " \n";
    //Log::infoLog() << "target: " << target << " \n";
    //Log::infoLog() << "ramp duration: " << ramp->duration() << " \n";
    //Log::infoLog() << "traj duration: " << traj->duration() << " \n";
    data._traj = traj;
    data._target = target;
    _bodyMap[body]._enabled = true;
    _bodies.push_back(body);

}

void BodyController::setTarget(Body* body, rw::trajectory::Trajectory<Transform3D<> >::Ptr traj, State& state){
    TargetData& data = _bodyMap[body];
    data.reset();
    data._type = TrajectoryController;
    data._traj = traj;
    data._target = traj->x( traj->endTime() );
    _bodyMap[body]._enabled = true;
    _bodies.push_back(body);
}

rw::math::Transform3D<> BodyController::getTarget(rwsim::dynamics::Body *body){
    if( _bodyMap.find(body)!=_bodyMap.end() ){
        // add body to control
        return _bodyMap[body]._target;
    }
    return Transform3D<>::identity();
}

Trajectory<Transform3D<> >::Ptr BodyController::getTargetTrajectory(Body *body){
    return _bodyMap[body]._traj;
}
