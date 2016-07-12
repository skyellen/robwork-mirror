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

#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>

#include "ODEUtil.hpp"
#include "ODESimulator.hpp"

#include <ode/ode.h>

#include <boost/bind.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim;


ODEBody::ODEBody(dBodyID odeBody, RigidBody::Ptr rwbody,
                 rw::math::Vector3D<> offset, int matID, int conID):
                _mframe(rwbody->getMovableFrame()),
                _bodyId(odeBody),
                _body(rwbody),
                _rwframe(_mframe),
                _type(ODEBody::RIGID),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID),
                _rwBody(rwbody),
                _kBody(NULL),
                _offset(offset)
{
    _body->changedEvent().add( boost::bind(&ODEBody::bodyChangedListener, this, _1), this);
}

ODEBody::ODEBody(dBodyID odeBody, KinematicBody::Ptr kbody, int matID, int conID):
				_mframe(kbody->getMovableFrame()),
				_bodyId(odeBody),
				_body(kbody),
				_rwframe(kbody->getBodyFrame()),
				_type(ODEBody::KINEMATIC),
				_contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID),
				_rwBody(NULL),
				_kBody(kbody),
                _offset(0,0,0)
{
    _body->changedEvent().add( boost::bind(&ODEBody::bodyChangedListener, this, _1), this);
}

ODEBody::ODEBody(std::vector<dGeomID> geomIds, dynamics::Body::Ptr body, int matID, int conID):
                _mframe(NULL),
                _bodyId(0), // a fixed object in ODE is allways part of the 0 body
                _body(body),
                _rwframe(body->getBodyFrame()),
                _type(ODEBody::FIXED),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID),
                _geomId(geomIds[0]),
                _geomIds(geomIds),
                _rwBody(NULL),
                _kBody(NULL)
{
    _body->changedEvent().add( boost::bind(&ODEBody::bodyChangedListener, this, _1), this);
}

ODEBody::ODEBody(dBodyID odeBody, dynamics::Body::Ptr body, rw::math::Vector3D<> offset,
		int matID, int conID, ODEBodyType type):
                _mframe(NULL),
                _bodyId(odeBody), // a fixed object in ODE is allways part of the 0 body
                _body(body),
                _rwframe(body->getBodyFrame()),
                _type( type ),
                _contactReductionThreshold(0.005),// 1cm
                _materialID(matID),
                _contactID(conID),
                _rwBody(NULL),
                _kBody(NULL),
                _offset(offset)
{
    _rwBody = body.cast<RigidBody>();
    _body->changedEvent().add( boost::bind(&ODEBody::bodyChangedListener, this, _1), this);
}

ODEBody::ODEBody(dBodyID odeBody, rw::kinematics::Frame* frame):
    _mframe(dynamic_cast<MovableFrame*>(frame)),
    _bodyId(odeBody),
    _body(NULL),
    _rwframe(frame),
    _type(ODEBody::RigidDummy),
    _contactReductionThreshold(0.005),// 1cm
    _materialID(-1),
    _contactID(-1),
    _rwBody(NULL),
    _kBody(NULL)
{

}

void ODEBody::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
    //double dt = info.dt;
    switch(_type){
    case(ODEBody::RIGID): {
        Vector3D<> f = _rwBody->getForceW( state );
        Vector3D<> t = _rwBody->getTorqueW( state );
        _lastForce = f;
        dBodyAddForce(_bodyId, (dReal)f[0], (dReal)f[1], (dReal)f[2]);
        dBodyAddTorque(_bodyId, (dReal)t[0], (dReal)t[1], (dReal)t[2]);
    }
    break;
    case(ODEBody::RIGIDODE): {
        Vector3D<> f = _body->getForceW( state );
        Vector3D<> t = _body->getTorqueW( state );
        _lastForce = f;
        dBodyAddForce(_bodyId, (dReal)f[0], (dReal)f[1], (dReal)f[2]);
        dBodyAddTorque(_bodyId, (dReal)t[0], (dReal)t[1], (dReal)t[2]);
    }
    break;
    case(ODEBody::RigidDummy): {
        return;
    }
    break;
    case(ODEBody::KINEMATIC): {

        // set the position of the kinematic body
        //Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        //wTb.P() += wTb.R()*_offset;
        //ODEUtil::setODEBodyT3D( _bodyId, wTb );

        Vector3D<> avel = _body->getAngVelW(state);
        Vector3D<> lvel = _body->getLinVelW(state);
        //std::cout << "kbody vel: " << lvel  << " " << avel << std::endl;
        dBodySetAngularVel(_bodyId, avel[0], avel[1], avel[2]);
        dBodySetLinearVel(_bodyId, lvel[0], lvel[1], lvel[2]);
        //std::cout << "kasdk" << std::endl;
        // TODO: calculate the velocity of this object and set it.
    }
    break;
    case(ODEBody::FIXED): {

    }
    break;
    /*
    case(ODEBody::RIGIDJOINT): {
        Vector3D<> f = _body->getForceW( state );
        Vector3D<> t = _body->getTorqueW( state );
        std::cout << "RIGIDJOINT:" << f << t << std::endl;
        _lastForce = f;
        dBodySetForce(_bodyId, (dReal)f[0], (dReal)f[1], (dReal)f[2]);
        dBodySetTorque(_bodyId, (dReal)t[0], (dReal)t[1], (dReal)t[2]);
    }
    break;
    */
    default:
        RW_WARN("UNSUPPORTED ODEBody type");
        break;
    }
}


void ODEBody::postupdate(rw::kinematics::State& state){
	//std::cout << _body->getName() << std::endl;

	switch(_type){
    case(ODEBody::RIGID): {
        //std::cout << _mframe->getName() << std::endl;
        Transform3D<> wTp = rw::kinematics::Kinematics::worldTframe( _mframe->getParent(state), state);
        //std::cout << "wTp    : " << wTp << std::endl;
        //std::cout << "wTb_ode: " << ODEUtil::getODEBodyT3D(_bodyId) << std::endl;

        Transform3D<> pTb = inverse(wTp) * ODEUtil::getODEBodyT3D(_bodyId);
        pTb.P() -= pTb.R()*_offset;
        //std::cout << "pTb" << pTb << std::endl;
        _mframe->setTransform( pTb , state );
        //std::cout << "pTb" << _mframe->getTransform( state ) << std::endl;


        //_rwBody->setWorldTcom(ODEUtil::getODEBodyT3D(_bodyId), state);
        Vector3D<> ang = ODEUtil::toVector3D( dBodyGetAngularVel(_bodyId) );
        Vector3D<> lin = ODEUtil::toVector3D( dBodyGetLinearVel(_bodyId) );

        // angular velocity is defined in world coordinates and around center of mass
        //if(_rwBody==NULL)
        //    std::cout << "BODY is null" << std::endl;
        if(_rwBody!=NULL){
            _rwBody->setAngVelW( ang , state);
            _rwBody->setLinVelW( lin , state);
        }

        //_rwBody->setForce( Vector3D<>::zero(), state );
        //_rwBody->setTorque( Vector3D<>::zero(), state );
    }
    break;
    case(ODEBody::RIGIDODE): {
        // nothing can really be done here, since we don't know anything about the
    }
    break;
    case(ODEBody::KINEMATIC): {
    	if(_mframe!=NULL){
			Transform3D<> wTp = rw::kinematics::Kinematics::worldTframe( _mframe->getParent(state), state);
			Transform3D<> pTb = inverse(wTp) * ODEUtil::getODEBodyT3D(_bodyId);
			pTb.P() -= pTb.R()*_offset;
			_mframe->setTransform( pTb , state );
			//Vector3D<> ang = ODEUtil::toVector3D( dBodyGetAngularVel(_bodyId) );
			//Vector3D<> lin = ODEUtil::toVector3D( dBodyGetLinearVel(_bodyId) );

			//_kBody->setAngVel( ang );
			//_kBody->setLinVel( lin );
    	}
    }
    break;
    case(ODEBody::RigidDummy): {
        return;
    }
    break;
    case(ODEBody::FIXED): {
        // the fixed bodies are not allowed to move during simulation so we don't expect any changes here
    }
    break;
    default:
        RW_WARN("UNSUPPORTED ODEBody type");
        break;
    }
    // reset force accumulation
    _body->setForce( Vector3D<>::zero(), state );
    _body->setTorque( Vector3D<>::zero(), state );
}

void ODEBody::setTransform(const rw::kinematics::State& state){
    Transform3D<> wtb = Kinematics::worldTframe(_rwframe, state);
    setTransform( wtb );
}

void ODEBody::setTransform(const rw::math::Transform3D<>& wTbody){
    if(_type==FIXED){
        // fixed object only has geometries. These may be offset individually
        BOOST_FOREACH(ODEUtil::TriGeomData* gdata, _triGeomDatas){
        	if (gdata->isPlaceable) {
        		Transform3D<> gt3d = wTbody * gdata->t3d;
        		ODEUtil::setODEGeomT3D(gdata->geomId, gt3d);
        	}
        }
    } else {
        Transform3D<> wTcom = wTbody;
        wTcom.P() += wTbody.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTcom );
    }
}

void ODEBody::setTransformCOM(const rw::math::Transform3D<>& wTcom){
    if(_type==FIXED){
        // fixed object only has geometries. These may be offset individually
        BOOST_FOREACH(ODEUtil::TriGeomData* gdata, _triGeomDatas){
        	if (gdata->isPlaceable) {
        		Transform3D<> gt3d = wTcom * gdata->t3d;
        		ODEUtil::setODEGeomT3D(gdata->geomId, gt3d);
        	}
        }
    } else {
        ODEUtil::setODEBodyT3D( _bodyId, wTcom );
    }
}

rw::math::Transform3D<> ODEBody::getTransform() const {
    if(_type==FIXED){
        if(_triGeomDatas.size()>0 && _triGeomDatas[0]->isPlaceable){
            Transform3D<> wTgeom_off = ODEUtil::getODEGeomT3D(_triGeomDatas[0]->geomId);
            return wTgeom_off * inverse(_triGeomDatas[0]->t3d);
        }
        return ODEUtil::getODEGeomT3D(_geomId);
    } else {
        rw::math::Transform3D<> wTb = ODEUtil::getODEBodyT3D(_bodyId);
        wTb.P() -= wTb.R()*_offset;
        return wTb;
    }
}

rw::math::Transform3D<> ODEBody::getTransformCOM() const {
    if(_type==FIXED){
        if(_triGeomDatas.size()>0 && _triGeomDatas[0]->isPlaceable){
            Transform3D<> wTgeom_off = ODEUtil::getODEGeomT3D(_triGeomDatas[0]->geomId);
            return wTgeom_off * inverse(_triGeomDatas[0]->t3d);
        }
        return ODEUtil::getODEGeomT3D(_geomId);
    } else {
        rw::math::Transform3D<> wTb = ODEUtil::getODEBodyT3D(_bodyId);
        return wTb;
    }
}



void ODEBody::bodyChangedListener(dynamics::Body::BodyEventType eventtype){
    //std::cout << "BODY Changed event"  << std::endl;
    switch(_type){
    case(ODEBody::RIGID): {
        BodyInfo info = _rwBody->getInfo();
        ODEUtil::setODEBodyMass(_bodyId, info.mass, Vector3D<>(0,0,0), info.inertia);
    }
    break;
    case(ODEBody::KINEMATIC): {

    }
    break;
    case(ODEBody::FIXED): {
        // TODO: run through all fixed objects and set their configuration
    }
    break;
    case(ODEBody::RigidDummy): {
        return;
    }
    break;
    default:
        RW_WARN("UNSUPPORTED ODEBody type");
        break;
    }


}

void ODEBody::reset(const rw::kinematics::State& state){
	switch(_type){
    case(ODEBody::RIGID): {
    	Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _mframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );
        Vector3D<> avel = _body->getAngVelW(state);
        Vector3D<> lvel = _body->getLinVelW(state);
        //std::cout << "kbody vel: " << lvel  << " " << avel << std::endl;
        dBodyEnable( _bodyId );
        dBodySetAngularVel(_bodyId, avel[0], avel[1], avel[2]);
        dBodySetLinearVel(_bodyId, lvel[0], lvel[1], lvel[2]);
    }
    break;
    case(ODEBody::RigidDummy): {
        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );

        dBodyEnable( _bodyId );
        dBodySetAngularVel(_bodyId, 0, 0, 0);
        dBodySetLinearVel(_bodyId, 0, 0, 0);
        return;
    }
    break;
    case(ODEBody::RIGIDODE): {
        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );

        Vector3D<> avel = _body->getAngVelW(state);
        Vector3D<> lvel = _body->getLinVelW(state);
        //std::cout << "kbody vel: " << lvel  << " " << avel << std::endl;
        dBodyEnable( _bodyId );
        dBodySetAngularVel(_bodyId, avel[0], avel[1], avel[2]);
        dBodySetLinearVel(_bodyId, lvel[0], lvel[1], lvel[2]);
    }
    break;
    case(ODEBody::KINEMATIC): {
        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );
        //std::cout << _rwframe->getName() << std::endl;
        //std::cout << _kBody->getName() << std::endl;

        Vector3D<> avel = _body->getAngVelW(state);
        Vector3D<> lvel = _body->getLinVelW(state);
        //std::cout << "kbody vel: " << lvel  << " " << avel << std::endl;
        dBodySetAngularVel(_bodyId, avel[0], avel[1], avel[2]);
        dBodySetLinearVel(_bodyId, lvel[0], lvel[1], lvel[2]);
    }
    break;
    case(ODEBody::FIXED): {
        // TODO: run through all fixed objects and set their configuration
        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( _rwframe, state);
        wTb.P() += wTb.R()*_offset;

        BOOST_FOREACH(ODEUtil::TriGeomData* gdata, _triGeomDatas){
        	if (gdata->isPlaceable) {
        		Transform3D<> gt3d = wTb * gdata->t3d;
        		ODEUtil::setODEGeomT3D(gdata->geomId, gt3d);
        	}
        }
    }
    break;
    default:
    	RW_WARN("UNSUPPORTED ODEBody type");
    	break;
	}
}


ODEBody* ODEBody::makeRigidBody(dynamics::Body::Ptr rwbody,  dSpaceID spaceId, ODESimulator* sim){
    State state = sim->getDynamicWorkCell()->getWorkcell()->getDefaultState();

    const BodyInfo& info = rwbody->getInfo();
    std::vector<Geometry::Ptr> geoms = rwbody->getGeometry();
    std::vector<ODEUtil::TriGeomData*> gdatas = ODEUtil::buildTriGeom(geoms, spaceId, rwbody->getBodyFrame(), state, false);

    if(gdatas.size()==0){
        RW_WARN("Body: "<< rwbody->getBodyFrame()->getName() << " has no geometry!");
    }

    Vector3D<> mc = info.masscenter;
    // create the body and initialize mass, inertia and stuff

    dBodyID bodyId = dBodyCreate( sim->getODEWorldId() );
    //ODEUtil::setODEBodyT3D(bodyId, rwbody->wTcom(state) );
    ODEUtil::setODEBodyMass(bodyId, info.mass, Vector3D<>(0,0,0), info.inertia);

    int mid = sim->getMaterialMap().getDataID( info.material );
    int oid = sim->getContactMap().getDataID( info.objectType );

    ODEBody *odeBody=0;

    // now if the rwbody is of type RigidBody then use this
    if( RigidBody::Ptr rbody = rwbody.cast<RigidBody>() ){
        odeBody = new ODEBody(bodyId, rbody, info.masscenter, mid, oid);
        dBodySetData (bodyId, (void*)odeBody);

    // } else if( ) {  // add more body types here

    } else {
        odeBody = new ODEBody(bodyId, rwbody, info.masscenter, mid, oid, ODEBody::RIGIDODE);
        dBodySetData (bodyId, (void*)odeBody);
    }

    // check if body frame has any properties that relate to ODE
    if( rwbody->getBodyFrame()->getPropertyMap().has("LinearDamping") ){
        dReal linDamp = rwbody->getBodyFrame()->getPropertyMap().get<double>("LinearDamping");
        dBodySetLinearDamping(bodyId, linDamp);
    }
    if( rwbody->getBodyFrame()->getPropertyMap().has("AngularDamping") ){
        dReal angDamp = rwbody->getBodyFrame()->getPropertyMap().get<double>("AngularDamping");
        dBodySetAngularDamping(bodyId, angDamp);
    }

    // now associate all geometry with the body
    BOOST_FOREACH(ODEUtil::TriGeomData* gdata, gdatas){
    	if (!gdata->isPlaceable)
    		RW_THROW("ODE can not use Plane geometry for rigid objects!");
        odeBody->getTriGeomData().push_back(gdata);
        //Vector3D<> mc = gdata->t3d.R() * bmc;
        dGeomSetBody(gdata->geomId, bodyId);
        dGeomSetData(gdata->geomId, odeBody);
        //_frameToOdeGeoms[rwbody->getBodyFrame()] = gdata->geomId;
        // the geom must be attached to body before offset is possible
        dGeomSetOffsetPosition(gdata->geomId, gdata->p[0]-mc[0], gdata->p[1]-mc[1], gdata->p[2]-mc[2]);
        dGeomSetOffsetQuaternion(gdata->geomId, gdata->rot);
    }
    //dBodySetMaxAngularSpeed(bodyId, 10);

    //_rwODEBodyToFrame[odeBody] = rwbody->getBodyFrame();
    //_rwFrameToODEBody[rwbody->getBodyFrame()] = odeBody;
    //BOOST_FOREACH(Frame* frame, rwbody->getFrames()){
        //std::cout  << "--> Adding frame: " << frame->getName() << std::endl;
    //    _rwFrameToODEBody[frame] = odeBody;
    //}
    sim->addODEBody(odeBody);
    return odeBody;
}


ODEBody* ODEBody::makeKinematicBody(Body::Ptr kbody, dSpaceID spaceid, ODESimulator *sim)
{
    RW_ASSERT(kbody!=NULL);
    State state = sim->getDynamicWorkCell()->getWorkcell()->getDefaultState();
    BodyInfo info = kbody->getInfo();

    // create a triangle mesh for all statically connected nodes
    std::vector<Geometry::Ptr> geoms = kbody->getGeometry();
    std::vector<ODEUtil::TriGeomData*> gdatas = ODEUtil::buildTriGeom(geoms, spaceid, kbody->getBodyFrame(), state,false);
    // if no triangles was loaded then continue
    if( gdatas.size()==0 ){
        RW_WARN("No triangle mesh defined for this body: " << kbody->getBodyFrame()->getName());
    }

    Vector3D<> mc = info.masscenter;
    //Transform3D<> wTb = Kinematics::worldTframe(kbody->getBodyFrame(), state);
    //wTb.P() += wTb.R()*mc;

    dBodyID bodyId = dBodyCreate( sim->getODEWorldId() );
    dBodySetKinematic(bodyId);
    //ODEUtil::setODEBodyT3D(bodyId, wTb);

    int mid = sim->getMaterialMap().getDataID( info.material );
    int oid = sim->getContactMap().getDataID( info.objectType );
    ODEBody *odeBody;
    if( KinematicBody::Ptr rwkbody = kbody.cast<KinematicBody>() ){
        odeBody = new ODEBody(bodyId, rwkbody, mid , oid);
    } else {
        odeBody = new ODEBody(bodyId, kbody, mc,  mid , oid, ODEBody::KINEMATIC);
    }

    //_odeBodies.push_back(odeBody);
    dBodySetData (bodyId, odeBody);
    //_allbodies.push_back(bodyId);
    //_rwODEBodyToFrame[odeBody] = kbody->getBodyFrame();
    //_rwFrameToODEBody[kbody->getBodyFrame()] = odeBody;

    BOOST_FOREACH(ODEUtil::TriGeomData* gdata, gdatas){
    	if (!gdata->isPlaceable)
    		RW_THROW("ODE can not use Plane geometry for kinematic objects!");

        odeBody->getTriGeomData().push_back(gdata);

        dGeomSetBody(gdata->geomId, bodyId);
        dGeomSetData(gdata->geomId, odeBody);

        //_frameToOdeGeoms[kbody->getBodyFrame()] = gdata->geomId;

        // set position and rotation offset of the geometry relative to the body
        dGeomSetOffsetPosition(gdata->geomId, gdata->p[0]-mc[0], gdata->p[1]-mc[1], gdata->p[2]-mc[2]);
        dGeomSetOffsetQuaternion(gdata->geomId, gdata->rot);
    }

    //BOOST_FOREACH(Frame* frame, kbody->getFrames()){
    //    RW_DEBUGS( "(KB) --> Adding frame: " << frame->getName() );
    //    _rwFrameToODEBody[frame] = odeBody;
    //}
    sim->addODEBody(odeBody);
    return odeBody;
}

ODEBody* ODEBody::makeFixedBody(Body::Ptr rwbody, dSpaceID spaceid, ODESimulator *sim)
{
    State state = sim->getDynamicWorkCell()->getWorkcell()->getDefaultState();
    const BodyInfo& info = rwbody->getInfo();
    //FixedBody *rbody = dynamic_cast<FixedBody*>( rwbody );
    //if(rbody==NULL)
    //    RW_THROW("Not a fixed body!");
    // create a triangle mesh for all statically connected nodes
    std::vector<Geometry::Ptr> geoms = rwbody->getGeometry();
    std::vector<ODEUtil::TriGeomData*> gdatas = ODEUtil::buildTriGeom(geoms, sim->getODESpace(), rwbody->getBodyFrame(), state, false);
    // if no triangles was loaded then continue
    if( gdatas.size()==0 ){
        RW_WARN("No triangle mesh defined for this body: " << rwbody->getBodyFrame()->getName());
    }

    //Vector3D<> mc = info.masscenter;
    //Transform3D<> wTb = Kinematics::worldTframe(rbody->getBodyFrame(), state);

    // create vector of geomids
    std::vector<dGeomID> geomids(gdatas.size());
    for(size_t i=0; i<gdatas.size(); i++ ){
        geomids[i] = gdatas[i]->geomId;
    }

    int mid = sim->getMaterialMap().getDataID( info.material );
    int oid = sim->getContactMap().getDataID( info.objectType );
    ODEBody *odeBody = new ODEBody(geomids, rwbody, mid , oid);

    BOOST_FOREACH(ODEUtil::TriGeomData* gdata, gdatas){
        odeBody->getTriGeomData().push_back(gdata);
        // set position and rotation of body
        dGeomSetData(gdata->geomId, odeBody);
        if (gdata->isPlaceable) {
        	Transform3D<> gt3d = /*wTb* */ gdata->t3d;
        	ODEUtil::setODEGeomT3D(gdata->geomId, gt3d);
        }
    }

    return odeBody;
}
