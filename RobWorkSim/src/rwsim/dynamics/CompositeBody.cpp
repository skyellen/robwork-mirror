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

#include "RigidBody.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;

using namespace rwsim::dynamics;

#define MIN_MASS_SIZE 0.0000001

namespace {

    /*ConstraintNode::NodeType getTypeImpl(double mass){
        if( mass<MIN_MASS_SIZE )
            return ConstraintNode::Fixed;
        return ConstraintNode::Rigid;
    }
*/
    double getInvMassImpl(double mass){
        if( mass<MIN_MASS_SIZE )
            return 0.0;
        return 1.0/mass;
    }

}
/*
RigidBody::RigidBody(
    const BodyInfo& info,
    BodyIntegrator &integrator,
    MovableFrame& frame,
    const std::vector<Frame*>& frames,
    rw::kinematics::State& state
    ):
        Body( getTypeImpl(info.mass), frame, frames),
        _mass( info.mass ),
        _massInv( getInvMassImpl(info.mass) ),
        _integrator(&integrator),
        _mframe( frame ),
        _materialID(info.material),
        _parent( *frame.getParent(state) ),
        _bodyType(0),
        _Ibody(info.inertia),
        _IbodyInv( inverse(info.inertia) ),
        _pTb( frame.getTransform(state) ),
        _IInv(info.inertia),
        _info(info)
{

};*/

RigidBody::RigidBody(
    const BodyInfo& info,
    MovableFrame* frame,
    const std::vector<Geometry::Ptr>& geoms,
    rw::kinematics::State& state
    ):
        Body(6, info, frame , geoms), // we use 6 dof in state data to hold ang and lin velocity
        _mass( info.mass ),
        _massInv( getInvMassImpl(info.mass) ),
        _mframe( frame ),
        _parent( frame->getParent(state) ),
        _bodyType(0),
        _Ibody(info.inertia),
        _IbodyInv( inverse(info.inertia) )
{

};


void RigidBody::rollBack(State &state){
}

void RigidBody::saveState(double h, rw::kinematics::State& state){
}

rw::math::InertiaMatrix<> RigidBody::calcInertiaTensor(const rw::kinematics::State& state) const {
	Transform3D<> pTb = Kinematics::frameTframe(_parent, _mframe, state);
	return pTb.R() * (this->getInertia() * inverse(pTb.R()));
}

rw::math::InertiaMatrix<> RigidBody::calcInertiaTensorInv(const rw::kinematics::State& state) const{
	Transform3D<> pTb = Kinematics::frameTframe(_parent, _mframe, state);
	return pTb.R() * (inverse(this->getInertia()) * inverse(pTb.R()));
}

rw::math::InertiaMatrix<> RigidBody::calcEffectiveMassW(const rw::math::Vector3D<>& wPc, const rw::kinematics::State& state) const{
	Transform3D<> wTb = getWTBody(state);
	Vector3D<> ra = wPc - wTb.P();
	 InertiaMatrix<> iInv = wTb.R() * (inverse(this->getInertia()) * inverse(wTb.R()));
	 Rotation3D<> skewra = Rotation3D<>::skew(ra);
	 InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
	 double massInv = getMassInv();
	 K(0,0) += massInv;
	 K(1,1) += massInv;
	 K(2,2) += massInv;
	 return K;
}

rw::math::InertiaMatrix<> RigidBody::calcEffectiveMass(const rw::math::Vector3D<>& pPc, const rw::kinematics::State& state) const{
	Transform3D<> pTb = _mframe->getTransform(state);

	Vector3D<> ra = pPc - pTb.P();
	 InertiaMatrix<> iInv = pTb.R() * (inverse(this->getInertia()) * inverse(pTb.R()));
	 Rotation3D<> skewra = Rotation3D<>::skew(ra);
	 InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
	 double massInv = getMassInv();
	 K(0,0) += massInv;
	 K(1,1) += massInv;
	 K(2,2) += massInv;
	 return K;
}

double RigidBody::calcEnergy(const rw::kinematics::State& state){
	InertiaMatrix<> ITensor = calcInertiaTensor(state);
    Vector3D<> angVel = getAngVel(state);
    Vector3D<> linVel = getLinVel(state);
    double mass = getMass();
	double Iw2 = dot((ITensor*angVel), angVel);
    double energy = 0.5*(mass*dot(linVel,linVel)+ mass*9.8*getWTBody(state).P()[2]);
    return energy;
}

void RigidBody::addForceWToPosW(const rw::math::Vector3D<>& force,
                             const rw::math::Vector3D<>& pos,
                             rw::kinematics::State& state) {
	Transform3D<> pTw = inverse(Kinematics::worldTframe(_parent, state));
	Transform3D<> wTb = Kinematics::worldTframe(_mframe, state);

    // transform the force into body frame description
    rw::math::Vector3D<> forcebody = pTw.R() * force;

    // calculate the center force contribution
    _force += forcebody;

    rw::math::Vector3D<> posOnBody = pTw.R() * (pos - wTb.P());

    // calculate the torque contribution
    _torque += cross( posOnBody, forcebody );
}
/*
void RigidBody::addImpulseWToPosW(const rw::math::Vector3D<>& impulse,
                       const rw::math::Vector3D<>& pos){

    // transform the force into body frame description
    rw::math::Vector3D<> ibody = _pTw.R() * impulse;

    // calculate the center force contribution
    _linImpulse += ibody;

    rw::math::Vector3D<> posOnBody = _pTw.R() * (pos - _wTb.P());

    // calculate the torque contribution
    _angImpulse += cross( posOnBody , ibody );
}
*/
rw::math::Vector3D<> RigidBody::getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const {
	Transform3D<> wTp = Kinematics::worldTframe(_parent, state);
	Transform3D<> wTb = Kinematics::worldTframe(_mframe, state);
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = inverse(wTp).R() * (p - wTb.P());
    // then calculate the velocity of the point relative to the body frame
    rw::math::Vector3D<> pVelBody = getLinVel(state) + cross(getAngVel(state), posOnBody);
    // adn last remember to transform velocity back to world frame
    return wTp.R() * pVelBody;
}

void RigidBody::resetState(rw::kinematics::State &state){
    rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
    this->setForce(zeroVec, state);
    this->setTorque(zeroVec, state);
    this->setAngVel(zeroVec, state);
    this->setLinVel(zeroVec, state);
}


void RigidBody::setAngVel(const rw::math::Vector3D<> &avel, rw::kinematics::State& state){
    double *q = this->getData(state);
    q[3] = avel[0];
    q[4] = avel[1];
    q[5] = avel[2];
}

void RigidBody::addForceW(const rw::math::Vector3D<>& force, rw::kinematics::State& state){
    addForce( inverse(Kinematics::worldTframe(_parent, state)).R() * force, state );
}

void RigidBody::setForceW(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
    setForce( inverse(Kinematics::worldTframe(_parent, state)).R() * f, state);
}
