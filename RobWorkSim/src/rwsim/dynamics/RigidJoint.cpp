/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RigidJoint.hpp"

#ifdef zkdaslkdiasdn
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsim::dynamics;

#define MIN_MASS_SIZE 0.0000001

namespace {

    double getInvMassImpl(double mass){
        if( mass<MIN_MASS_SIZE )
            return 0.0;
        return 1.0/mass;
    }

}

RigidJoint::RigidJoint(
    const BodyInfo& info,
    rw::models::Joint* joint,
    rw::models::Object::Ptr obj,
    rw::kinematics::State& state
    ):
        Body(joint->getDOF(), info, obj),
        _mass( info.mass ),
        _massInv( getInvMassImpl(info.mass) ),
        _frame( joint ),
        _Ibody(info.inertia),
        _IbodyInv( inverse(info.inertia) ),
        _pTb( joint->getTransform(state) ),
        _IInv(info.inertia),
        _joint(joint)
{

};

rw::math::InertiaMatrix<> RigidJoint::getEffectiveMassW(const rw::math::Vector3D<>& wPc){
	 Vector3D<> ra = wPc - getWTBody().P();
	 //std::cout << "ra" << std::endl;
	 // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/mass
	 // Skew calculates the cross-product matrix from a vector
	 //std::cout << "massInv " << massInv << std::endl;
	 InertiaMatrix<> iInv = getInertiaTensorInvW();
	 Rotation3D<> skewra = Rotation3D<>::skew(ra);
	 InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
	 double massInv = getInvMass();
	 K(0,0) += massInv;
	 K(1,1) += massInv;
	 K(2,2) += massInv;
	 return K;
}

void RigidJoint::addForceWToPosW(const rw::math::Vector3D<>& force,
                             const rw::math::Vector3D<>& pos){

    // transform the force into body frame description
    rw::math::Vector3D<> forcebody = _pTw.R() * force;

    // calculate the center force contribution
    _force += forcebody;

    rw::math::Vector3D<> posOnBody = _pTw.R() * (pos - _wTb.P());

    // calculate the torque contribution
    _torque += cross( posOnBody, forcebody );
}

rw::math::Vector3D<> RigidJoint::getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const{
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = _pTw.R() * (p - _wTb.P());
    // then calculate the velocity of the point relative to the body frame
    rw::math::Vector3D<> pVelBody = _linVel + cross(_angVel, posOnBody);
    // adn last remember to transform velocity back to world frame
    return _wTp.R() * pVelBody;
}

void RigidJoint::reset(rw::kinematics::State &state){
    rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
    _force = zeroVec;
    _torque = zeroVec;
    _linVel = zeroVec;
    _angVel = zeroVec;
    //_mframe.setTransform( _pTb , state );
    _wTb = rw::kinematics::Kinematics::worldTframe( _joint, state);
    _bTw = inverse( _wTb );

//    _wTp = rw::kinematics::Kinematics::WorldTframe( &_parent, state);
    _pTw = inverse( _wTp );
}
#endif
