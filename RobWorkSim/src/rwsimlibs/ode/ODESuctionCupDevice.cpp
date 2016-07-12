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


#include "ODESuctionCupDevice.hpp"

#include "ODEBody.hpp"
#include "ODESimulator.hpp"

#include <rw/geometry/TriMesh.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>

#include <ode/ode.h>
#include <vector>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;


#include <rw/geometry.hpp>

//using namespace rwsim::control;
using namespace rwlibs::control;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::proximity;

namespace {
    /**
     * @brief generates a plate with spikes of length \b d sticking out in the z-axis. The spikes are ordered in
     * a nr of rings on the plate. First ring starts at radius \b r and last ring ends in r/n where n is nr of rings.
     * And for each ring there is M equally spaced spikes.
     *
     * @return
     */
    PlainTriMesh<>::Ptr makeSpikedCup(double d, double r, size_t N, size_t M){
        PlainTriMesh<>::Ptr mesh = ownedPtr(new PlainTriMesh<>());

        Vector3D<> spike = Vector3D<>::z() * d;
        for(size_t i = 0; i<N; i++){
            double radius = r-(r*i/N);
            Vector3D<> xvec = Vector3D<>::x()*radius;
            for(size_t j=0; j<M; j++){
                // we rotate the x vector around the z axis to get the coordinate
                Rotation3D<> rotZ = RPY<>(2*Pi/M*j,0,0).toRotation3D();
                Vector3D<> coord = rotZ * xvec;

                // we create the triangle
                Triangle<> tri( coord, coord+normalize(xvec)*0.00001, coord+spike );
                mesh->add(tri);
            }
        }

        return mesh;
    }

}

const int NR_OF_SPIKES = 10;

ODESuctionCupDevice::ODESuctionCupDevice(ODEBody *base,
                                         rwsim::dynamics::SuctionCup* dev,
                                         ODESimulator *odesim,
                                         State& state):
        _dev(dev), //_sensor(sensor),
        _isInContact(false),
        _odesim(odesim),
        _worldId(odesim->getODEWorldId() ),
        _fjoint(NULL),
        _lastX(0),
        _lastAng(0)
{
    //std::vector<dynamics::RigidJoint*> joints = _dev->getRigidJoints();
    //_tcp = joints.back();
    init(base, dev, odesim, state);

    // create the collision detector
    _narrowStrategy = ownedPtr( new rwlibs::proximitystrategies::ProximityStrategyPQP() );

    // create the spiked cup geometry
    _spikedCupMesh = makeSpikedCup(odesim->getMaxSeperatingDistance()+0.002, dev->getRadius(), 1, NR_OF_SPIKES);

    _spikedCup = ownedPtr(new Geometry(_spikedCupMesh));

    _spikedCupModel = _narrowStrategy->createModel(  );
    _spikedCupModel->addGeometry( *_spikedCup );

    _contactGroupId = dJointGroupCreate(0);


}

ODESuctionCupDevice::~ODESuctionCupDevice(){

}

void ODESuctionCupDevice::updateNoRollBack(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
    //double dt = info.dt;
    // test if tcp is in contact with object
    /// std::cout <<  "ODESuctionCupDevice" << std::endl;
    // if it is in sufficient contact then apply attracting forces to object
    Body *object = NULL;

    //bool inContact = false;
    std::vector<Body::Ptr> cbodies = _sensor->getBodies(state);
    std::vector<Contact3D> contacts = _sensor->getContacts(state);

    // test if the entire mouthpiece is in contact
    //std::cout <<  "Contacts: " << contacts.size() << std::endl;
    int cidx = 0; //, contactIdx =0;
    if(contacts.size()>0){
        BOOST_FOREACH(Body::Ptr b, cbodies ){
            if(b!=NULL){
                object = b.get();
                //contactIdx = cidx;
            }
            cidx++;
        }
    }

    //double pressure = _dev->getPressure(state);
    //bool isVacuumOn = pressure>0.001;
    bool firstContact = false;
    // we only use the contacts to initiate a contact scenario. When we know that an object is sucked on
    // then we use our own collision stuff to determine contact.
    //std::cout << (!_isInContact) <<  "&&"  << (object!=NULL) << std::endl;
    if( !_isInContact && object!=NULL){
        //std::cout << "!_isInContact && object!=NULL" << object->getBodyFrame()->getName() << std::endl;
        Transform3D<> wTobj = Kinematics::worldTframe(object->getBodyFrame(), state);
        Transform3D<> wTcup = Kinematics::worldTframe(_tcp->getBodyFrame(), state);
        // test if the suction gripper is in "complete" contact with the object

        _pdata.setCollisionQueryType(rw::proximity::CollisionStrategy::AllContacts);

        ProximityModel::Ptr objModel = _narrowStrategy->getModel( object->getBodyFrame() );
        if(objModel==NULL){
            _narrowStrategy->addModel( object->getObject() );
            objModel = _narrowStrategy->getModel( object->getBodyFrame() );
        }


        if( _narrowStrategy->inCollision(objModel, wTobj, _spikedCupModel, wTcup, _pdata) ){

            CollisionResult& result = _pdata.getCollisionData();

            //std::cout << result._geomPrimIds.size() << ">" << NR_OF_SPIKES-1 << std::endl;
            if( result._geomPrimIds.size()>NR_OF_SPIKES-1 ){
                // tjek that all contacts are within a certain distance
                // 1. get the trimesh of object
                std::vector<bool> spikeContact(NR_OF_SPIKES, false);
                //std::vector<Geometry::Ptr> geoms = object->getGeometry();
                BOOST_FOREACH(CollisionResult::CollisionPair pair, result._collisionPairs){
                    //Geometry::Ptr geom = geoms[ pair.geoIdxA ];
                    //TriMesh::Ptr mesh = geom->getGeometryData().cast<TriMesh>();
                    //if(mesh==NULL)
                    //    continue;

                    // for now we just check if all spikes are in contact
                    for(int i=pair.startIdx;i<pair.startIdx+pair.size;i++){
                        size_t idx = result._geomPrimIds[i].second;
                        RW_ASSERT(idx>=0);
                        RW_ASSERT((int)idx<NR_OF_SPIKES);
                        spikeContact[idx] = true;
                    }
                }

                _object = object;
                _isInContact = true;
                firstContact = true;

                BOOST_FOREACH(bool incontact, spikeContact){
                    if(!incontact){
                        _isInContact = false;
                        firstContact = false;
                        _object = NULL;
                    }
                }

                //std::cout << "_isInContact: " << _isInContact << std::endl;

            }
        }
    } else if(_isInContact ){
        /*
        Transform3D<> wTobj = Kinematics::worldTframe(_object->getBodyFrame(), state);
        Transform3D<> wTcup = Kinematics::worldTframe(_tcp->getBodyFrame(), state);

        // make sure we detect when its no longer in contact
        ProximityModel::Ptr objModel = _narrowStrategy->getModel( _object->getBodyFrame() );

        // rotate z into y by rotating 90 degree around x-axis
        //Transform3D<> yTz( RPY<>(0,0,Pi/2).toRotation3D() );

        if( _narrowStrategy->inCollision(objModel, wTobj, _spikedCupModel, wTcup, _pdata) ){
            if( _pdata.getCollisionData()._geomPrimIds.size()<NR_OF_SPIKES-1 ){
                std::cout << " Contact lost, only: " << _pdata.getCollisionData()._geomPrimIds.size() << " contacts" << std::endl;
                _object = NULL;
                _isInContact = false;
                dJointGroupEmpty(_contactGroupId);
            }
        }
        */
    }

    if( firstContact ) {

        // we connect the two bodies using a 6DOF fixed constraint
        // TODO: first move the bodies into a close position
        //_odesim->attach(_odeEnd->getRwBody(), object);
        _fjoint = dJointCreateFixed(_worldId, 0 );
        dJointAttach(_fjoint, _odeEnd->getBodyID(),  _odesim->getODEBodyId(_object.get()));
        dJointSetFixed(_fjoint);

        // disable collision checking between the two bodies
        _odesim->disableCollision( _odeEnd->getRwBody(), _object );

        // now add a sensor for monitoring the torque and force
        dJointSetFeedback(_fjoint, &_feedback);
        _dev->setClosed(true, state);
        _dev->setContactBody( _object.get(), state );
    } else if(_isInContact){
        // todo: monitor the forces on the fixed joint
        Vector3D<> force = ODEUtil::toVector3D( _feedback.f1 );
        //Vector3D<> torque = ODEUtil::toVector3D( _feedback.t1 );
        //std::cout << "Force;: " << force << " ... " << torque << std::endl;



        // since we simulate the contact using a rigid joint the user will not be
        // able to detect any contacts, therefore we have to add emulated contacts.
        // These will not be used in the physics engine but will apear as contacts
        // for the user if he requires it.
        double forceFromVacuum = 10; // normal
        Transform3D<> t3d = _tcp->getTransformW( state );
        Vector3D<> normal = t3d.R() * Vector3D<>( 0, 0, 1 );
        Vector3D<> maxforce = normal*forceFromVacuum;

        // the attracting joint
        //dJointID c;
        Vector3D<> xaxis = t3d.R()*Vector3D<>::x()*_dev->getRadius();
        Vector3D<> yaxis = t3d.R()*Vector3D<>::y()*_dev->getRadius();

        _odesim->addEmulatedContact(t3d.P()+xaxis, maxforce/4 , normal, _object.get());
        _odesim->addEmulatedContact(t3d.P()-xaxis, maxforce/4 , normal, _object.get());
        _odesim->addEmulatedContact(t3d.P()+yaxis, maxforce/4 , normal, _object.get());
        _odesim->addEmulatedContact(t3d.P()-yaxis, maxforce/4 , normal, _object.get());

        bool forceToHigh = force(2)>500;
        if(forceToHigh){
            std::cout << " Contact lost, only: " << _pdata.getCollisionData()._geomPrimIds.size() << " contacts" << std::endl;
            _dev->setClosed(false, state);
            _dev->setContactBody( NULL, state );
            _odesim->enableCollision( _odeEnd->getRwBody(), _object );

            dJointDestroy(_fjoint);
            _isInContact = false;
            _object = NULL;
        }


    } else {
       // nothing to do here

    }

}


void ODESuctionCupDevice::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
    double dt = info.dt;

    if( !info.rollback ){
        updateNoRollBack(info, state);
    }

    // if we are in a rollback then no state changes should happen
    Q sp1;
    if(_dev->isClosed(state)){// _isInContact ){
        sp1 = _dev->getSpringParamsClosed();
    } else {
        sp1 = _dev->getSpringParamsOpen();
    }
    double pos = dJointGetSliderPosition(_slider);
    //double ang1 = dJointGetHingeAngle(_hinge1);
    //double ang2 = dJointGetHingeAngle(_hinge2);
    //std::cout << pos << "m " << ang1*Rad2Deg << "Deg " << ang2*Rad2Deg << "Deg"<< std::endl;

    Transform3D<> wTbase = _dev->getBaseBody()->getTransformW(state);
    Transform3D<> wToff = wTbase * _dev->getOffset();
    Transform3D<> wTend = _dev->getEndBody()->getTransformW(state);
    Vector3D<> saxis = wToff.R()*Vector3D<>::z(); // slider axis is along the z-axis

    // find angle between saxis and the cupplane
    double ang = angle(saxis, wTend.R()*Vector3D<>::z() );

    // apply spring constants for linier spring
    //double h = 0;//_dev->getHeight();

    double x = sp1(4)-pos;
    double xd = _lastX-x;
    double linforce = sp1(0)*x - sp1(1)*xd/dt;
    Vector3D<> lf = normalize(saxis)*linforce;
    _odeEnd->getRwBody()->addForceW(lf, state);
    //dBodyAddForce( _odeEnd->getODEBody(), lf(0),lf(1),lf(2) );

    // now add angular spring forces
    double angD = _lastAng-ang;
    double angforce = sp1(2)*ang - sp1(3)*angD/dt;
    Vector3D<> rotAxis = wTend.R()*Vector3D<>::x();
    if(ang>0.1*Deg2Rad){
        rotAxis = -normalize( cross(normalize(saxis), wTend.R()*Vector3D<>::z() ) );
        //_odeEnd->getRwBody()->addTorqueW( rotAxis*angforce, state);
        Vector3D<> t = rotAxis*angforce;
        dBodyAddTorque(_odeEnd->getBodyID(),t[0],t[1],t[2]  );
    }
     //std::cout <<  x << "m "<< pos << "m " << linforce<< "N " <<  ang*Rad2Deg << "Deg "<< angforce << "Nm "<< std::endl;
    //std::cout <<  x << "m "<< pos << "m " << sp1(4) << "m " << std::endl;
    _lastX = x;
    _lastAng = ang;
}

void ODESuctionCupDevice::reset(rw::kinematics::State& state){
    // reset the suction cup to the current state

    Transform3D<> wTbase = Kinematics::worldTframe(_dev->getBase()->getBodyFrame(), state);
    Transform3D<> wToff = wTbase * _dev->getOffset();
    Transform3D<> wTend = Transform3D<>(wToff.P() + wToff.R()*( Vector3D<>::z()*_dev->getSpringParamsOpen()(4) ), wToff.R() );

    _odeBase->reset(state);
    ODEUtil::setODEBodyT3D(_bTmp1, wTend);
    ODEUtil::setODEBodyT3D(_bTmp2, wTend);
    _odeEnd->reset(state);
    //ODEUtil::setODEBodyT3D(_odeEnd->getODEBody(), wTend);
    _dev->setClosed(false, state);
    _dev->setContactBody( NULL, state );
    if(_isInContact){
        _odesim->enableCollision( _odeEnd->getRwBody(), _object );
        dJointDestroy(_fjoint);
    }

    _isInContact = false;
    _object = NULL;
    dJointGroupEmpty(_contactGroupId);

    double pos = dJointGetSliderPosition(_slider);
    //std::cout << "POS: " << pos << " == " << _dev->getHeight() << "-" << _dev->getSpringParamsOpen()(4) << std::endl;
    //double ang1 = dJointGetHingeAngle(_hinge1);
    //double ang2 = dJointGetHingeAngle(_hinge2);
    _lastAng = 0;
    _lastX = _dev->getSpringParamsOpen()(4)-pos;
}

void ODESuctionCupDevice::postUpdate(rw::kinematics::State& state){
    //dJointGroupEmpty(_contactGroupId);
}


void ODESuctionCupDevice::init(ODEBody *odebase, rwsim::dynamics::SuctionCup* scup, ODESimulator *sim, rw::kinematics::State &state){
    // create base

    BodyInfo info = scup->getEndBody()->getInfo();
    Body::Ptr base = scup->getBase();
    _odeBase = odebase;
    RW_ASSERT(_odeBase);
    //_odeEnd = ODEBody::makeRigidBody( scup->getEndBody(), sim->getODESpace(), sim);
    _odeEnd = sim->getODEBody(scup->getEndBody()->getBodyFrame());//ODEBody::makeRigidBody( scup->getEndBody(), sim->getODESpace(), sim);
    _ode_bodies.push_back(_odeBase);
    _ode_bodies.push_back(_odeEnd);

    _tcp = _odeEnd->getRwBody();
    // we create a couple of kinematic bodies to lie in between the joints
    dBodyID bTmp1 = dBodyCreate( sim->getODEWorldId() );
    dBodyID bTmp2 = dBodyCreate( sim->getODEWorldId() );
    _bTmp1 = bTmp1;
    _bTmp2 = bTmp2;

    ODEUtil::setODEBodyMass(bTmp1,info.mass, Vector3D<>(0,0,0), info.inertia );
    ODEUtil::setODEBodyMass(bTmp2,info.mass, Vector3D<>(0,0,0), info.inertia );

    //dBodySetKinematic(bTmp1);
    //dBodySetKinematic(bTmp2);
    sim->addODEBody(bTmp1);
    sim->addODEBody(bTmp2);

    Transform3D<> wTbase = Kinematics::worldTframe(base->getBodyFrame(), state);
    Transform3D<> wToff = wTbase * scup->getOffset();
    Transform3D<> wTend = Transform3D<>(wToff.P() + wToff.R()*( Vector3D<>::z()*scup->getSpringParamsOpen()(4) ), wToff.R() );
    Vector3D<> saxis =  wToff.R()*(Vector3D<>::z()); // slider axis is along the z-axis

    // we want the zero position to be at wTbase
    ODEUtil::setODEBodyT3D(_odeBase->getBodyID(), wTbase);
    ODEUtil::setODEBodyT3D(bTmp1, wTbase);
    ODEUtil::setODEBodyT3D(bTmp2, wTbase);
    ODEUtil::setODEBodyT3D(_odeEnd->getBodyID(), wTbase);

    // now create all constraints that we use for the cup
    dJointID slider = dJointCreateSlider (sim->getODEWorldId(), 0);
    dJointAttach(slider, bTmp1, _odeBase->getBodyID());
    dJointSetSliderAxis(slider, saxis(0) , saxis(1), saxis(2));
    double lostop = scup->getSpringParamsClosed()(4);
    //double highstop = scup->getSpringParamsOpen()(4);
    dJointSetSliderParam(slider, dParamLoStop, lostop-0.01 );
    //dJointSetSliderParam(slider, dParamHiStop, highstop+0.01 );
    dJointSetSliderParam(slider, dParamCFM, 0.01);

    sim->addODEJoint(slider);

    // we set the joint to be in open configuration
    ODEUtil::setODEBodyT3D(bTmp1, wTend);
    ODEUtil::setODEBodyT3D(bTmp2, wTend);
    ODEUtil::setODEBodyT3D(_odeEnd->getBodyID(), wTend);

    Vector3D<> xaxis = wToff.R()*Vector3D<>::x();
    Vector3D<> yaxis = wToff.R()*Vector3D<>::y();
    Vector3D<> hpos = wToff*( Vector3D<>::z()*scup->getSpringParamsOpen()(4) );

    dJointID hinge1 = dJointCreateHinge (sim->getODEWorldId(), 0);
    dJointAttach(hinge1, bTmp2, bTmp1);
    dJointSetHingeAxis(hinge1, xaxis(0) , xaxis(1), xaxis(2));
    dJointSetHingeAnchor(hinge1, hpos(0), hpos(1), hpos(2));
    dJointSetHingeParam(hinge1, dParamCFM, 0.001);

    //dJointSetHingeParam(hinge1, dParamLoStop, -60*Deg2Rad );
    //dJointSetHingeParam(hinge1, dParamHiStop,  60*Deg2Rad );

    sim->addODEJoint(hinge1);

    dJointID hinge2 = dJointCreateHinge (sim->getODEWorldId(), 0);
    dJointAttach(hinge2, _odeEnd->getBodyID(), bTmp2);
    dJointSetHingeAxis(hinge2, yaxis(0) , yaxis(1), yaxis(2));
    dJointSetHingeAnchor(hinge2, hpos(0), hpos(1), hpos(2));
    dJointSetHingeParam(hinge2, dParamCFM, 0.001);
    sim->addODEJoint(hinge2);

    //dJointSetHingeParam(hinge2, dParamLoStop, -60*Deg2Rad );
    //dJointSetHingeParam(hinge2, dParamHiStop,  60*Deg2Rad );

    _slider = slider;
    _hinge1 = hinge1;
    _hinge2 = hinge2;

    // last we make the BodyContactSensor
    _sensor = ownedPtr(new BodyContactSensor("SuctionCupDevSensor", _odeEnd->getRwBody()->getBodyFrame()));
    sim->addSensor(_sensor, state);

    /*
    dJointID motor = dJointCreateLMotor (sim->getODEWorldId(), 0);
    dJointAttach(motor, _odeBase->getODEBody(), odeParent);
    dJointSetLMotorNumAxes(motor, 1);
    dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
    //dJointSetLMotorAngle(motor,0, qinit);
    dJointSetLMotorParam(motor,dParamFMax, maxForce(i) );
    dJointSetLMotorParam(motor,dParamVel,0);

    //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
    //dJointSetAMotorParam(Amotor,dParamHiStop,0);
    ODEJoint *odeJoint = new ODEJoint(ODEJoint::Prismatic, slider, motor, odeChild, rjoint);
    _jointToODEJoint[pjoint] = odeJoint;
    odeJoints.push_back(odeJoint);
    _allODEJoints.push_back(odeJoint);
     */
}



#ifdef CONTACT_BASED_SUC_CUP

    //if( _isInContact ){
    if( firstContact ) {
        // apply forces to object
        std::cout << " first contact " << std::endl;
        double forceFromVacuum = 10; // normal
        double forceCupFromVacuum = -1; // normal
        Transform3D<> t3d = _tcp->getTransformW( state );
        Vector3D<> normal = t3d.R() * Vector3D<>( 0, 0, 1 );

        //Vector3D<> objforce = t3d.R() * Vector3D<>( 0, 0, 1*forceFromVacuum);
        //Vector3D<> cupforce = t3d.R() * Vector3D<>( 0, 0, 1*forceCupFromVacuum);
        /// std::cout <<  "Object: " << _object->getBodyFrame()->getName() << std::endl;
        /// std::cout <<  "objForce: "  << objforce << " " << t3d.P() << std::endl;
        /// std::cout <<  "cupForce: "  << cupforce << " " << t3d.P() << std::endl;

        // create the contact point list
        _contacts.clear();
        dContact con;
        con.geom.depth = 0.0001;
        ODEUtil::toODEVector(t3d.P(),con.geom.pos);
        ODEUtil::toODEVector(normal,con.geom.normal);
        con.surface.mode =
                dContactBounce
                | dContactSoftCFM
                | dContactSoftERP;

        con.surface.bounce = 0;
        con.surface.bounce_vel = 0.0001;
        con.surface.mu = 0.5;
        con.surface.soft_cfm = 0.0001;
        con.surface.soft_erp = 0.2;


        // the attracting joint
        //dJointID c;
        Vector3D<> xaxis = t3d.R()*Vector3D<>::x()*_dev->getRadius();
        Vector3D<> yaxis = t3d.R()*Vector3D<>::y()*_dev->getRadius();

        ODEUtil::toODEVector(t3d.P()+xaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()-xaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()+yaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()-yaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        // the real contact joints
        ODEUtil::toODEVector(-normal,con.geom.normal);

        ODEUtil::toODEVector(t3d.P()+xaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()-xaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()+yaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        ODEUtil::toODEVector(t3d.P()-yaxis,con.geom.pos);
        _contacts.push_back(con);
        //c = dJointCreateContact (_odesim->getODEWorldId(), _contactGroupId, &con);
        //dJointAttach (c, _odesim->getODEBodyId(_tcp.get()), _odesim->getODEBodyId(_object.get()) );

        //Vector3D<> force = Vector3D<>( 0, 0, 1*forceFromVacuum );
        //std::cout << force << std::endl;
        //_object->addForceWToPosW(objforce, t3d.P(), state);
        //_tcp->addForceWToPosW(cupforce, t3d.P(), state);
    }

    //ODEBody *odeobject = _odesim->getODEBody(object->getBodyFrame());
    //RW_ASSERT()
    if(_contacts.size()>0 && _object!=NULL){
        std::cout << "adding contacts: " << _contacts.size() << std::endl;
        _odesim->addContacts(_contacts, _contacts.size(), _odeEnd, _odesim->getODEBody(_object->getBodyFrame()));
    }
    //Q _elasticity(3);
    //_elasticity(0) = 100/0.017; // total compression is 0.017 where a maximum force of 50 should be resisted, so
    //_elasticity(1) = 0.5/(40*Deg2Rad); // total compression is 40 degree where a maximum torque of X should be resisted
    //_elasticity(2) = 0.5/(40*Deg2Rad); // total compression is 40 degree where a maximum torque of Y should be resisted
    // now update the elastic forces of the gripper
    //Q q = _dev->getQ(state);
    //Q ql = _dev->getForceLimit();

    //for(size_t i=0;i<q.size();i++){
    //    ql(i) = fabs( q(i)*_elasticity(i) );
    //}
    //std::cout << "Force lim: " << ql << std::endl;
    //_dev->setForceLimit(ql/10);

    //std::cout << "ODESuctionCupDevice END" << std::endl;
#endif
