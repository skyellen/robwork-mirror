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

#include "ODEBody.hpp"

#include "ODEUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>

#include <ode/ode.h>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

ODEBody::ODEBody(dBodyID odeBody,
            RigidBody* rwbody,
            rw::math::Vector3D<> offset, int matID, int conID):
                _bodyId(odeBody),
                _rwBody(rwbody),
                _mframe(rwbody->getMovableFrame()),
                _offset(offset),
                _rwframe(_mframe),
                _type(ODEBody::RIGID),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID)
{
}

ODEBody::ODEBody(dBodyID odeBody,
            RigidJoint* rwbody,
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

ODEBody::ODEBody(dBodyID odeBody, KinematicBody* kbody, int matID, int conID):
				_mframe(NULL),
				_bodyId(odeBody),
				_kBody(kbody),
				_body(kbody),
				_offset(0,0,0),
				_rwframe(&kbody->getBodyFrame()),
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

        _rwBody->setAngVel( ang , state);
        _rwBody->setLinVel( lin , state);
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
	switch(_type){
    case(ODEBody::RIGID): {
    	Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _mframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );
    }
    case(ODEBody::RIGIDJOINT): {
    	//std::cout << "Reset rigid joint" << std::endl;
    	//Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        //wTb.P() += wTb.R()*_offset;
        //ODEUtil::setODEBodyT3D( _bodyId, wTb );
    }
    break;
    case(ODEBody::FIXED): {

    }
    break;
    default:
    	RW_WARN("UNSUPPORTED ODEBody type");
	}

    dBodyEnable( _bodyId );
    dBodySetAngularVel( _bodyId, 0, 0, 0 );
    dBodySetLinearVel( _bodyId, 0, 0, 0 );
}
