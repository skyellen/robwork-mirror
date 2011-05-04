#include "BodyController.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rwsim::control;
using namespace rwsim::dynamics;
using namespace rw::math;
using namespace rw::kinematics;


BodyController::BodyController(const std::string& name):
	Controller(name)
{
}

void BodyController::update(double dt, rw::kinematics::State& state) {
    //std::cout << "B" << std::endl;
    const double MAX_LIN_ACCELERATION = 0.05;

    BOOST_FOREACH(Body* body, _bodies){
        if( KinematicBody *kbody = dynamic_cast<KinematicBody*>(body) ){
            // set angular and linear velocities of body such that it will move toward target
            Transform3D<> wTt = _bodyMap[body];
            Transform3D<> wTb = Kinematics::worldTframe(kbody->getBodyFrame(), state);
            //std::cout << wTt << "\n" << wTb << std::endl;
            //std::cout << Quaternion<>(wTt.R()) << "\n" << Quaternion<>(wTb.R()) << std::endl;
            //std::cout << RPY<>(wTt.R()) << "\n" << RPY<>(wTb.R()) << std::endl;
            //std::cout << EAA<>(wTt.R()) << "\n" << EAA<>(wTb.R()) << std::endl;

            const Transform3D<>& bTt = inverse(wTb) * wTt;

            const VelocityScrew6D<> vel( bTt );
            const VelocityScrew6D<> velW = (wTb.R() * vel) * 10;

            Vector3D<> lastLinVel = kbody->getLinVelW( state );
            Vector3D<> vErr = velW.linear()-lastLinVel;
            double scale,linAcc = (vErr).normInf()/dt;
            if(linAcc>MAX_LIN_ACCELERATION)
                scale = MAX_LIN_ACCELERATION/linAcc;
            else
                scale = 1.0;
            // we need to limit the current velocity such that a constant deacceleration from now does not overshoot the target
            //lastLinVel

            Vector3D<> linVelW_target = lastLinVel+vErr*scale;
            kbody->setLinVelW( linVelW_target , state);

            // and now control the angular velocity
            Vector3D<> lastAngVel = kbody->getAngVelW( state );
            Vector3D<> angVel(velW(3),velW(4),velW(5));
            //EAA<> e;
            double scale_ang, angAcc = angVel.normInf();
            if(angAcc>0.4)
                scale_ang = 0.4/angAcc;
            else
                scale_ang = 1.0;

            //std::cout << angVel*la << std::endl;
            kbody->setAngVelW(angVel*scale_ang, state);
        } else {

        }
    }

}

void BodyController::disableBodyControl(rwsim::dynamics::Body *body){
    if(_bodyMap.find(body)==_bodyMap.end())
        return;
    _bodyMap.erase( _bodyMap.find(body) );

    _bodies.erase( find(_bodies.begin(), _bodies.end(), body));

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
    if( _bodyMap.find(body)==_bodyMap.end() ){
        // add body to control
        _bodyMap[body] = target;
        _bodies.push_back(body);
    } else {
        _bodyMap[body] = target;
    }
}

rw::math::Transform3D<> BodyController::getTarget(rwsim::dynamics::Body *body){
    return Transform3D<>::identity();
}

