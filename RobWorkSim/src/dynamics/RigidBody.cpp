#include "RigidBody.hpp"

#include <boost/numeric/ublas/matrix.hpp>

#include <rw/math/LinearAlgebra.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::kinematics;
using namespace dynamics;
using namespace rw::geometry::sandbox;

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
    MovableFrame& frame,
    const std::vector<Geometry*>& geoms,
    rw::kinematics::State& state
    ):
        Body( info, &frame , geoms),
        _mass( info.mass ),
        _massInv( getInvMassImpl(info.mass) ),
        _mframe( frame ),
        _materialID(info.material),
        _parent( *frame.getParent(state) ),
        _bodyType(0),
        _Ibody(info.inertia),
        _IbodyInv( inverse(info.inertia) ),
        _pTb( frame.getTransform(state) ),
        _ITensorInv(info.inertia),
        _ITensor(info.inertia)
{

};


void RigidBody::rollBack(State &state){
    _torque = _torqueRB;
    _force = _forceRB;

    _linVel = _linVelRB;
    _angVel = _angVelRB;

    _wTb = Kinematics::worldTframe( &_mframe, state);
    _bTw = inverse( _wTb );

    _wTp = Kinematics::worldTframe( &_parent, state);
    _pTw = inverse( _wTp );
    _pTb = _mframe.getTransform(state);

    _linImpulse = Vector3D<>(0,0,0);
    _angImpulse = Vector3D<>(0,0,0);
}

void RigidBody::saveState(double h, rw::kinematics::State& state){
    // update rollBack variables
    _torqueRB = _torque;
    _forceRB = _force;

    _mframe.setTransform( _pTb , state );

    _linVelRB = _linVel;
    _angVelRB = _angVel;
}

void RigidBody::calcAuxVarialbles(rw::kinematics::State& state){
    _wTb = Kinematics::worldTframe( &_mframe, state);
    _bTw = inverse( _wTb );
    _wTp = Kinematics::worldTframe( &_parent, state);
    _pTw = inverse( _wTp );
    _pTb = _mframe.getTransform(state);

    _ITensorInv = _pTb.R() * (_IbodyInv * inverse(_pTb.R()));
    _ITensor = _pTb.R() * (_Ibody * inverse(_pTb.R()));

}

rw::math::InertiaMatrix<> RigidBody::getEffectiveMassW(const rw::math::Vector3D<>& wPc){
	 Vector3D<> ra = wPc - getWTBody().P();
	 //std::cout << "ra" << std::endl;
	 // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/mass
	 // Skew calculates the cross-product matrix from a vector
	 //std::cout << "massInv " << massInv << std::endl;
	 InertiaMatrix<> iInv = getInertiaTensorInvW();
	 Rotation3D<> skewra = Rotation3D<>::skew(ra);
	 InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
	 double massInv = getMassInv();
	 K(0,0) += massInv;
	 K(1,1) += massInv;
	 K(2,2) += massInv;
	 return K;
}

void RigidBody::updatePosition(double h, State &state){
	Vector3D<> pos = _pTb.P();
    //_integrator->updatePosition(h,_pTb.P(),_pTb.R(),_linVel,_angVel);
   // std::cout  << "Pos body: " << (_pTb.P()-pos) << std::endl;
    // update the state with rotation and position, calculate aux variables
    _mframe.setTransform( _pTb , state );

    _wTb = Kinematics::worldTframe( &_mframe, state);
    _bTw = inverse( _wTb );

    _wTp = Kinematics::worldTframe( &_parent, state);
    _pTw = inverse( _wTp );
}

void RigidBody::updateVelocity(double h, State &state){
    // Simple Euler integration
    // update position --- EULER STEP ---
    Vector3D<> linAcc = _force * _massInv;

    // Calculate the inertia matrix and its inverse
    _ITensorInv = _pTb.R() * (_IbodyInv * inverse(_pTb.R()));
    InertiaMatrix<> I = _pTb.R() * (_Ibody * inverse(_pTb.R()));

    //std::cout << "Update velocity using integrator: " << _linVel << std::endl;
    //_integrator->updateVelocity(h, _linVel, _angVel, linAcc, _torque, I, _IInv);

    // Dampen the velocity a bit Friction with air and such
    _angVel = _angVel - _angVel*0.05*h;
    _linVel = _linVel - _linVel*0.05*h;

    //std::cout << "Updated velocity: " << _linVel << std::endl;
}

double RigidBody::calcEnergy(){
    double Iw2 = dot((_ITensor*_angVel), _angVel);
    double energy = 0.5*(_mass*dot(_linVel,_linVel)+ _mass*9.8*_wTb.P()[2]);

    return energy;
}

void RigidBody::updateImpulse(){
    //std::cout << "Update impulse: " << _linImpulse << std::endl;
    _linVel += _linImpulse * _massInv;

    InertiaMatrix<> I = _pTb.R() * (_Ibody * inverse(_pTb.R()));
    Vector3D<> tau = cross( _angVel, I*_angVel );
    _angVel += _ITensorInv * (_angImpulse /* - tau*/ );

    _linImpulse = Vector3D<>(0.0,0.0,0.0);
    _angImpulse = Vector3D<>(0.0,0.0,0.0);
}

void RigidBody::addForceWToPosW(const rw::math::Vector3D<>& force,
                             const rw::math::Vector3D<>& pos){

    // transform the force into body frame description
    rw::math::Vector3D<> forcebody = _pTw.R() * force;

    // calculate the center force contribution
    _force += forcebody;

    rw::math::Vector3D<> posOnBody = _pTw.R() * (pos - _wTb.P());

    // calculate the torque contribution
    _torque += cross( posOnBody, forcebody );
}

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

rw::math::Vector3D<> RigidBody::getPointVelW(const rw::math::Vector3D<>& p){
    // first transform point to body frame
    rw::math::Vector3D<> posOnBody = _pTw.R() * (p - _wTb.P());
    // then calculate the velocity of the point relative to the body frame
    rw::math::Vector3D<> pVelBody = _linVel + cross(_angVel, posOnBody);
    // adn last remember to transform velocity back to world frame
    return _wTp.R() * pVelBody;
}

void RigidBody::resetState(rw::kinematics::State &state){
    rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
    _force = zeroVec;
    _torque = zeroVec;
    _linImpulse = zeroVec;
    _angImpulse = zeroVec;
    _linVel = zeroVec;
    _angVel = zeroVec;
    _mframe.setTransform( _pTb , state );
    _wTb = Kinematics::worldTframe( &_mframe, state);
    _bTw = inverse( _wTb );

    _wTp = Kinematics::worldTframe( &_parent, state);
    _pTw = inverse( _wTp );
    saveState(0.1,state);
}
