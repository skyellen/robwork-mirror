/*
 * ODEBody.cpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#include "ODEBody.hpp"

#include "ODEUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>

#include <ode/ode.h>

using namespace rw::kinematics;
using namespace rw::math;

ODEBody::ODEBody(dBodyID odeBody,
            dynamics::RigidBody* rwbody,
            rw::math::Vector3D<> offset, int matID, int conID):
                _bodyId(odeBody),
                _rwBody(rwbody),
                _mframe(&rwbody->getMovableFrame()),
                _offset(offset),
                _rwframe(_mframe),
                _type(ODEBody::RIGID),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID)
{
}

ODEBody::ODEBody(dBodyID odeBody,
            dynamics::RigidJoint* rwbody,
            rw::math::Vector3D<> offset, int matID, int conID):
                _bodyId(odeBody),
                _body(rwbody),
                _offset(offset),
                _type(ODEBody::RIGIDJOINT),
                _rwframe(&rwbody->getBodyFrame()),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID)
{
}

ODEBody::ODEBody(dBodyID odeBody, dynamics::KinematicBody* kbody, int matID, int conID):
				_bodyId(odeBody),
				_kBody(kbody),
				_offset(0,0,0),
				_rwframe(_mframe),
				_type(ODEBody::KINEMATIC),
				_contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID)
{
}

ODEBody::ODEBody(dGeomID geomId, rw::kinematics::Frame* frame, int matID, int conID):
                _mframe(NULL),
                _bodyId(NULL),
                _geomId(geomId),
                _rwframe(frame),
                _type(ODEBody::FIXED),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID)
{
}

void ODEBody::update(double dt, rw::kinematics::State& state){
    switch(_type){
    case(ODEBody::RIGID): {
    }
    break;
    case(ODEBody::KINEMATIC): {
    	// TODO: calculate the velocity of this object and set it.

        //Transform3D<> wTp = rw::kinematics::Kinematics::worldTframe( _mframe->getParent(), state);
        //Transform3D<> pTb = inverse(wTp) * ODEUtil::getODEBodyT3D(_bodyId);
        //pTb.P() -= pTb.R()*_offset;
        //_mframe->setTransform( pTb , state );

        //Vector3D<> ang = ODEUtil::toVector3D( dBodyGetAngularVel(_bodyId) );
        //Vector3D<> lin = ODEUtil::toVector3D( dBodyGetLinearVel(_bodyId) );

        //_kBody->setAngVel( ang );
        //_kBody->setLinVel( lin );
    }
    break;
    case(ODEBody::FIXED): {

    }
    break;
    case(ODEBody::RIGIDJOINT): {

    }
    break;
    default:
        RW_WARN("UNSUPPORTED ODEBody type");
    }
}


void ODEBody::postupdate(rw::kinematics::State& state){
    switch(_type){
    case(ODEBody::RIGID): {
        Transform3D<> wTp = rw::kinematics::Kinematics::worldTframe( _mframe->getParent(), state);
        Transform3D<> pTb = inverse(wTp) * ODEUtil::getODEBodyT3D(_bodyId);
        pTb.P() -= pTb.R()*_offset;
        _mframe->setTransform( pTb , state );

        Vector3D<> ang = ODEUtil::toVector3D( dBodyGetAngularVel(_bodyId) );
        Vector3D<> lin = ODEUtil::toVector3D( dBodyGetLinearVel(_bodyId) );

        _rwBody->setAngVel( ang );
        _rwBody->setLinVel( lin );
    }
    break;
    case(ODEBody::KINEMATIC): {
        //Transform3D<> wTp = rw::kinematics::Kinematics::worldTframe( _mframe->getParent(), state);
        //Transform3D<> pTb = inverse(wTp) * ODEUtil::getODEBodyT3D(_bodyId);
        //pTb.P() -= pTb.R()*_offset;
        //_mframe->setTransform( pTb , state );

        //Vector3D<> ang = ODEUtil::toVector3D( dBodyGetAngularVel(_bodyId) );
        //Vector3D<> lin = ODEUtil::toVector3D( dBodyGetLinearVel(_bodyId) );

        //_kBody->setAngVel( ang );
        //_kBody->setLinVel( lin );
    }
    break;
    case(ODEBody::FIXED): {

    }
    break;
    case(ODEBody::RIGIDJOINT): {

    }
    break;
    default:
        RW_WARN("UNSUPPORTED ODEBody type");
    }
}

void ODEBody::reset(const rw::kinematics::State& state){

	Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _mframe, state);
    wTb.P() += wTb.R()*_offset;
    ODEUtil::setODEBodyT3D( _bodyId, wTb );

    dBodyEnable( _bodyId );
    dBodySetAngularVel( _bodyId, 0, 0, 0 );
    dBodySetLinearVel( _bodyId, 0, 0, 0 );
}
