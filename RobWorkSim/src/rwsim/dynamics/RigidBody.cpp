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

#include <boost/numeric/ublas/matrix.hpp>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace boost::numeric;

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rwsim::dynamics;

#define MIN_MASS_SIZE 0.0000001

namespace {

    double getInvMassImpl(double mass){
        if( mass<MIN_MASS_SIZE )
            return 0.0;
        return 1.0/mass;
    }

}

RigidBody::RigidBody(const BodyInfo& info,rw::models::Object::Ptr obj):
        Body(info, obj), // we use 6 dof in state data to hold ang and lin velocity
        _mass( info.mass ),
        _massInv( getInvMassImpl(info.mass) ),
        _mframe( NULL ),
        _bodyType(0),
        _Ibody(info.inertia),
        _IbodyInv( inverse(info.inertia) ),
        _IbodyPrincipal(GeometryUtil::calculatePrincipalInertia(info.inertia))
        //_rstate(this)
{
    _mframe = dynamic_cast<MovableFrame*>(obj->getBase());
    if(_mframe==NULL){
        RW_THROW("Base frame of Object in a RigidBody must be a MovableFrame!");
    }

    add(_rstate);
};

rw::math::InertiaMatrix<> RigidBody::calcInertiaTensor(const rw::kinematics::State& state) const {
	Transform3D<> pTb = Kinematics::frameTframe(getParent(state), _mframe, state);
	return pTb.R() * (this->getInertia() * inverse(pTb.R()));
}

rw::math::InertiaMatrix<> RigidBody::calcInertiaTensorInv(const rw::kinematics::State& state) const{
	Transform3D<> pTb = Kinematics::frameTframe(getParent(state), _mframe, state);
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

double RigidBody::calcEnergy(const State& state, const Vector3D<>& gravity, const Vector3D<>& potZero) const {
	const InertiaMatrix<> ITensor = calcInertiaTensor(state);
    const Vector3D<> angVel = getAngVel(state);
    const Vector3D<> linVel = getLinVel(state);
    const double mass = getMass();
	const double Iw2 = dot((ITensor*angVel), angVel);
    const double energy = 0.5*(mass*dot(linVel,linVel) + Iw2) - mass*dot(gravity,getWTBody(state).P()-potZero);
    return energy;
}

rw::math::Vector3D<> RigidBody::getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const {
	Transform3D<> wTp = Kinematics::worldTframe(getParent(state), state);
	Transform3D<> wTb = Kinematics::worldTframe(_mframe, state);
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = inverse(wTp).R() * (p - wTb.P());
    // then calculate the velocity of the point relative to the body frame
    rw::math::Vector3D<> pVelBody = getLinVel(state) + cross(getAngVel(state), posOnBody);
    // adn last remember to transform velocity back to world frame
    return wTp.R() * pVelBody;
}

rw::math::VelocityScrew6D<> RigidBody::getVelocity(const rw::kinematics::State &state) const{
    Vector3D<> lv = getLinVel(state);
    Vector3D<> av = getAngVel(state);
    return VelocityScrew6D<>(lv[0],lv[1],lv[2],av[0],av[1],av[2]);
}

void RigidBody::reset(rw::kinematics::State &state){
    rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
    this->setForce(zeroVec, state);
    this->setTorque(zeroVec, state);
    this->setAngVel(zeroVec, state);
    this->setLinVel(zeroVec, state);
}

void RigidBody::setAngVel(const rw::math::Vector3D<> &avel, rw::kinematics::State& state){
    _rstate.get(state).angvel = avel;
}

