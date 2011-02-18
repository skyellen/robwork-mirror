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
    BOOST_FOREACH(Body* body, _bodies){
        if( KinematicBody *kbody = dynamic_cast<KinematicBody*>(body) ){
            // set angular and linear velocities of body such that it will move toward target
            Transform3D<> wTt = _bodyMap[body];
            Transform3D<> wTb = Kinematics::worldTframe(kbody->getBodyFrame(), state);
            const Transform3D<>& bTt = inverse(wTb) * wTt;

            const VelocityScrew6D<> vel( bTt );
            const VelocityScrew6D<> velW = (wTb.R() * vel) * 5;


            Vector3D<> linVel = (wTt.P()-wTb.P())*5;

            double la = linVel.normInf();
            if(la>0.1)
                la = 0.1/la;
            else
                la = 1.0;
            //std::cout << "Body vels: " << linVel*la << std::endl;
            kbody->setLinVel( linVel*la , state);

            /*
            double la = velW.linear().normInf();
            if(la>0.1)
                la = 0.1/la;
            else
                la = 1.0;
            kbody->setLinVel( velW.linear()*la , state);
        */
            //kbody->setLinVel( Math::clamp(velW.linear(),Vector3D<>(-0.1,-0.1,-0.1),Vector3D<>(-0.1,-0.1,-0.1)) , state);
            //Vector3D<> angVel(velW(3),velW(4),velW(5));
            //kbody->setAngVel(angVel, state);
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

