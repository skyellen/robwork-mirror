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

#include "ODEJoint.hpp"

#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>

#include <ode/ode.h>

#include "ODESimulator.hpp"

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rwsim;
using namespace rwsim::simulator;

namespace {

    //! return joint and motor
    std::pair<dJointID, dJointID> makeRevoluteJoint(RevoluteJoint* joint, ODEBody *parent, ODEBody *child, ODESimulator *sim, const State& state){
        // everything in ODE is defined in world coordinates
        Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
        Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
        Vector3D<> hpos = wTchild.P();

        std::pair<Q, Q> posBounds = joint->getBounds();
        const double qinit = joint->getData(state)[0];
        dJointID hinge = dJointCreateHinge(sim->getODEWorldId(), 0);
        dJointAttach(hinge, child->getBodyID(), parent->getBodyID());
        dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
        dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));
        dJointSetHingeParam(hinge, dParamCFM, 0.0001);

        // set the position limits
        // TODO: these stops can only handle in interval [-Pi, Pi]
        dJointSetHingeParam(hinge, dParamLoStop, posBounds.first[0] );
        dJointSetHingeParam(hinge, dParamHiStop, posBounds.second[0] );

        dJointID motor = NULL;
        if( joint->isActive() ){
            motor = dJointCreateAMotor(sim->getODEWorldId(), 0);
            dJointAttach(motor, child->getBodyID(), parent->getBodyID());
            dJointSetAMotorNumAxes(motor, 1);
            dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
            dJointSetAMotorAngle(motor,0, qinit);
            dJointSetAMotorParam(motor,dParamFMax, 1 );
            dJointSetAMotorParam(motor,dParamVel,0);
        }
        return std::make_pair(hinge, motor);
    }

    //! return joint and motor
    boost::tuple<dJointID, dJointID, ODEJoint*> makeDependentRevoluteJoint(DependentRevoluteJoint* joint, ODEBody *parent, ODEBody *child, ODESimulator *sim, const State& state){
        // everything in ODE is defined in world coordinates

        Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
        Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
        Vector3D<> hpos = wTchild.P();

        std::pair<Q, Q> posBounds = joint->getBounds();

        Joint *owner = &joint->getOwner();
        const double qinit = owner->getData(state)[0]*joint->getScale()+0;

        dJointID hinge = dJointCreateHinge(sim->getODEWorldId(), 0);
        dJointAttach(hinge, child->getBodyID(), parent->getBodyID());
        dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
        dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));
        dJointSetHingeParam(hinge, dParamCFM, 0.0001);

        // set the position limits
        // TODO: these stops can only handle in interval [-Pi, Pi]
        //dJointSetHingeParam(hinge, dParamLoStop, posBounds.first[0] );
        //dJointSetHingeParam(hinge, dParamHiStop, posBounds.second[0] );

        dJointID motor = NULL;
        if( joint->isActive() ){
            motor = dJointCreateAMotor(sim->getODEWorldId(), 0);
            dJointAttach(motor, child->getBodyID(), parent->getBodyID());
            dJointSetAMotorNumAxes(motor, 1);
            dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
            dJointSetAMotorAngle(motor,0, qinit);
            dJointSetAMotorParam(motor,dParamFMax, 20 );
            dJointSetAMotorParam(motor,dParamVel,0);
        }

        ODEJoint *odeOwner = sim->getODEJoint(owner);

        return boost::make_tuple(hinge, motor, odeOwner);
    }


    std::pair<dJointID, dJointID> makePrismaticJoint(PrismaticJoint* joint, ODEBody *parent, ODEBody *child, ODESimulator *sim, const State& state){
        // everything in ODE is defined in world coordinates
        Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
        Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
        //Vector3D<> hpos = wTchild.P();

        std::pair<Q, Q> posBounds = joint->getBounds();
        //const double qinit = joint->getData(state)[0];

        // test if another joint is dependent on this joint
        //const double qinit = pjoint->getData(state)[0];
        dJointID slider = dJointCreateSlider (sim->getODEWorldId(), 0);
        dJointAttach(slider, child->getBodyID(), parent->getBodyID());
        dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));
        //dJointSetHingeAnchor(slider, hpos(0), hpos(1), hpos(2));
        dJointSetSliderParam(slider, dParamLoStop, posBounds.first[0] );
        dJointSetSliderParam(slider, dParamHiStop, posBounds.second[0] );

        dJointID motor = NULL;
        if( joint->isActive() ){
            motor = dJointCreateLMotor (sim->getODEWorldId(), 0);
            dJointAttach(motor, child->getBodyID(), parent->getBodyID());
            dJointSetLMotorNumAxes(motor, 1);
            dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
            //dJointSetLMotorAngle(motor,0, qinit);

            dJointSetLMotorParam(motor,dParamFMax, 1 );
            dJointSetLMotorParam(motor,dParamVel,0);
        }

        return std::make_pair(slider, motor);
    }

    boost::tuple<dJointID, dJointID, ODEJoint*, ODEJoint::ODEJointType>  makeDependentPrismaticJoint(DependentPrismaticJoint* joint, ODEBody *parent, ODEBody *child, ODESimulator *sim, const State& state){
        // everything in ODE is defined in world coordinates
        Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
        Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
        //Vector3D<> hpos = wTchild.P();
        Joint *owner = &joint->getOwner();

        Transform3D<> wTowner = Kinematics::worldTframe(joint,state);

        Vector3D<> haxis_owner = wTowner.R() * Vector3D<>(0,0,1);

        std::pair<Q, Q> posBounds = joint->getBounds();
        //const double qinit = joint->getData(state)[0];

        ODEJoint::ODEJointType type;

        dJointID slider;
        dJointID motor;
        ODEJoint *odeOwner = sim->getODEJoint(owner);
        RW_ASSERT(odeOwner!=NULL);

        // test if the axes of owner and joint is parallel
        double ang = angle(haxis, haxis_owner );
        //std::cout << "THE ANGLE: " << ang << std::endl;
        if( (fabs(ang-180*Deg2Rad)<0.0001 || fabs(ang)<0.0001)&& dynamic_cast<PrismaticJoint*>(owner)!=NULL ) {
            //std::cout << "JOINTS ARE PARALLEL" << std::endl;
            type = ODEJoint::DEPEND_PAR;

            slider = dJointCreateSlider (sim->getODEWorldId(), 0);
            //dJointAttach(slider, odeChild, odeParent);
            dJointAttach(slider, child->getBodyID(), odeOwner->getChild()->getBodyID());
            dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));

            motor = dJointCreateLMotor (sim->getODEWorldId(), 0);
            //dJointAttach(motor, odeChild, odeParent);
            dJointAttach(motor, child->getBodyID(), odeOwner->getChild()->getBodyID());
            dJointSetLMotorNumAxes(motor, 1);
            dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));

           // std::cout << "i:" << i << " mforce_len: " << maxForce.size() << std::endl;
            // TODO: should take the maxforce value of the owner joint
            dJointSetLMotorParam(motor,dParamFMax, 20);
            dJointSetLMotorParam(motor,dParamVel, 0);

        } else {
            //std::cout << "JOINTS ARE NOT PARALLEL" << std::endl;
            // create ordinary
            type = ODEJoint::DEPEND;

            // test if another joint is dependent on this joint
            //const double qinit = pjoint->getData(state)[0];
            slider = dJointCreateSlider (sim->getODEWorldId(), 0);

            dJointAttach(slider, child->getBodyID(), parent->getBodyID());

            dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));

            //dJointSetHingeAnchor(slider, hpos(0), hpos(1), hpos(2));

            // bounds are dictated by the controlling joint so we leave them here
            //dJointSetSliderParam(slider, dParamLoStop, posBounds.first[0] );
            //dJointSetSliderParam(slider, dParamHiStop, posBounds.second[0] );

            // the dependency is controlled using a motor
            motor = dJointCreateLMotor (sim->getODEWorldId(), 0);

            dJointAttach(motor, child->getBodyID(), parent->getBodyID());

            dJointSetLMotorNumAxes(motor, 1);

            dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));

            //dJointSetLMotorAngle(motor,0, qinit);

            dJointSetLMotorParam(motor,dParamFMax, 10 );
            dJointSetLMotorParam(motor,dParamVel,0);

        }

        return boost::make_tuple(slider, motor, odeOwner ,type);
    }


}


ODEJoint::ODEJoint(rw::models::Joint* rwjoint,
                   ODEBody* parent,
                   ODEBody* child,
                   rwsim::simulator::ODESimulator *sim,
                   const rw::kinematics::State& state):
                   _rwJoint(rwjoint),
                   _owner(NULL),
                   _parent(parent),
                   _child(child)
{

    if( RevoluteJoint *rjoint=dynamic_cast<RevoluteJoint*>(rwjoint) ){

        boost::tie(_jointId,_motorId) = makeRevoluteJoint(rjoint, parent, child, sim, state);
        _type = ODEJoint::RIGID;
        _jtype = ODEJoint::Revolute;
    } else if(DependentRevoluteJoint *drjoint=dynamic_cast<DependentRevoluteJoint*>(rwjoint)){

        boost::tie(_jointId,_motorId,_owner) = makeDependentRevoluteJoint(drjoint, parent, child, sim, state);
        _type = ODEJoint::DEPEND;
        _jtype = ODEJoint::Revolute;
        _scale = drjoint->getScale();
        _off = drjoint->getOffset();
    } else if(PrismaticJoint *pjoint=dynamic_cast<PrismaticJoint*>(rwjoint)){

        boost::tie(_jointId,_motorId) = makePrismaticJoint(pjoint, parent, child, sim, state);
        _type = ODEJoint::RIGID;
        _jtype = ODEJoint::Prismatic;
    } else if(DependentPrismaticJoint *dpjoint=dynamic_cast<DependentPrismaticJoint*>(rwjoint)){

        boost::tie(_jointId,_motorId,_owner,_type) = makeDependentPrismaticJoint(dpjoint, parent, child, sim, state);
        _jtype = ODEJoint::Prismatic;
        _scale = dpjoint->getScale();
        _off = dpjoint->getOffset();
    } else {
        RW_THROW("Unsupported joint type!");
    }


}
/*

ODEJoint::ODEJoint(
		JointType jtype,
         dJointID odeJoint,
         dJointID odeMotor,
         dBodyID body,
         dynamics::RigidJoint* rwjoint):
			 _jtype(jtype),
             _jointId(odeJoint),
             _child->getBodyID()(body),
             _motorId(odeMotor),
             _rwJoint(rwjoint),
             _owner(NULL),
             _type(ODEJoint::RIGID),
             _bodyFrame(rwjoint->getBodyFrame()),
             _offset( rwjoint->getInfo().masscenter )
{

}

ODEJoint::ODEJoint(
		 JointType jtype,
		 dJointID odeJoint,
         dJointID odeMotor,
         dBodyID body,
         ODEJoint* owner,
         rw::kinematics::Frame *bframe,
         double scale,
         double off,
         dynamics::RigidJoint* rwjoint):
			 _jtype(jtype),
             _jointId(odeJoint),
             _motorId(odeMotor),
             _child->getBodyID()(body),
             _owner(owner),
             _scale(scale),
             _rwJoint(rwjoint),
             _off(off),
             _type(ODEJoint::DEPEND),
             _bodyFrame(bframe),
             _offset(rwjoint->getInfo().masscenter)
{

}
*/
/*
ODEJoint* ODEJoint::make(RevoluteJoint* joint, dBodyID parent, dWorldID worldId){
    const double qinit = rwjoint->getQ(initState)[0];

    dJointID hinge = dJointCreateHinge(worldId, 0);
    dJointAttach(hinge, odeChild, odeParent);
    dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
    dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

    dJointID motor = dJointCreateAMotor (_worldId, 0);
    dJointAttach(motor, odeChild, odeParent);
    dJointSetAMotorNumAxes(motor, 1);
    dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
    dJointSetAMotorAngle(motor,0, qinit);
    dJointSetAMotorParam(motor,dParamFMax, maxForce(i) );
    dJointSetAMotorParam(motor,dParamVel,0);

    //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
    //dJointSetAMotorParam(Amotor,dParamHiStop,0);
    //std::cout << "CREATED ODEJOINT: " << rwjoint->getName() << std::endl;
    ODEJoint *odeJoint = new ODEJoint(hinge, motor, rjoint);
    _jointToODEJoint[rwjoint] = odeJoint;
    odeJoints.push_back(odeJoint);

}
*/

void ODEJoint::reset(const rw::kinematics::State& state){
    // if the fixed transform between two bodies is changed
    // then any constraint between these need to be reset
    // so we need to reattach the constraint
    //Frame *bframe = NULL;
    State rstate = state;
    double zeroq[] = {0.0,0.0,0.0,0.0};
    if(_type!=ODEJoint::DEPEND){
        _rwJoint->setData(rstate, zeroq);
    } else {
        //_rwJoint->getJoint()->setData(rstate, zeroq);
        _owner->getJoint()->setData(rstate, zeroq);
    }
    _child->setTransform(rstate);
    _parent->setTransform(rstate);

    Transform3D<> wTchild = Kinematics::worldTframe(_rwJoint, rstate);
    Vector3D<> hpos = wTchild.P();
    Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);

    if(_jtype==Revolute){
        //dJointGetBody()
        dJointSetHingeAxis(_jointId, haxis(0) , haxis(1), haxis(2));
        dJointSetHingeAnchor(_jointId, hpos(0), hpos(1), hpos(2));
    } else if(_jtype==Prismatic){
        //dJointAttach(slider, odeChild->getBodyID(), odeParent->getBodyID());
        //dJointSetSliderAxis(_jointId, haxis(0) , haxis(1), haxis(2));
        //dJointSetSliderAnchor(_jointId, hpos(0), hpos(1), hpos(2));
    }

    _child->setTransform(state);
    _parent->setTransform(state);
    dBodyEnable( _child->getBodyID() );

    //! TODO: these should be set to the correct velocities defined in state

    //Vector3D<> avel = _child->getRwBody()->getAngVelW(state);
    //Vector3D<> lvel = _child->getRwBody()->getLinVelW(state);
    //dBodySetAngularVel( _child->getBodyID(), avel[0], avel[1], avel[2] );
    //dBodySetLinearVel( _child->getBodyID(), lvel[0], lvel[1], lvel[2] );
    dBodySetAngularVel( _child->getBodyID(), 0,0,0 );
    dBodySetLinearVel( _child->getBodyID(), 0,0,0 );

}

