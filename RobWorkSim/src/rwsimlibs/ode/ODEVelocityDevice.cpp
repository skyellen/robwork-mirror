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

#include "ODEVelocityDevice.hpp"


#include <rw/math/MetricUtil.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rwsim/dynamics/BodyUtil.hpp>
#include <boost/foreach.hpp>

#include "ODESimulator.hpp"
#include "ODEJoint.hpp"

#include <ode/ode.h>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

namespace {

    bool equalSign(double v1, double v2 ){
        return (v1<0 && v2<0) || (v1>0 && v2>0);
    }

}
/*
ODEVelocityDevice::ODEVelocityDevice(
            RigidDevice *rdev,
            std::vector<ODEJoint*> odeJoints,
            Q maxForce):
        _rdev(rdev),
        _odeJoints(odeJoints),
        _maxForce(maxForce)
{

}
*/
ODEVelocityDevice::~ODEVelocityDevice(){
	BOOST_FOREACH(ODEJoint* joint, _odeJoints){
		delete joint;
	}

	BOOST_FOREACH(ODEBody* body, _ode_bodies){
		delete body;
	}

}

void ODEVelocityDevice::reset(rw::kinematics::State& state){
    rw::math::Q q = _rdev->getModel().getQ(state);
    rw::math::Q flim = _rdev->getMotorForceLimits();
    int qi = 0;
    for(size_t i = 0; i<_odeJoints.size(); i++){

        _odeJoints[i]->setVelocity( 0 );

        bool depends = _odeJoints[i]->isDepend();
        if(depends){
            continue;
        }

        _odeJoints[i]->setAngle( q(qi) );
        _odeJoints[i]->setMaxForce( flim(qi) );

        qi++;
    }
    for(size_t i = 0; i<_odeJoints.size(); i++){

        bool depends = _odeJoints[i]->isDepend();

        if(depends){
            double angle = _odeJoints[i]->getOwner()->getAngle();
            _odeJoints[i]->setAngle(angle);
        }
        _odeJoints[i]->reset(state);
    }

}
namespace {
	int sign(double val){
		if(val<0)
			return -1;
		return 1;
	}
}


void ODEVelocityDevice::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
    double dt = info.dt;
    _lastDt = info.dt_prev;

    bool fmaxChanged = false;
    bool motormodesChanged = false;
    if(!info.rollback){
        // check if any control modes have been changed
        std::vector<RigidDevice::MotorControlMode> mstate = _rdev->getMotorModes( state );
        for(size_t i=0;i<mstate.size();i++){
            motormodesChanged |= mstate[i]!=_modes[i];
            _modes[i] = mstate[i];
        }

        // check if force limits have changed
        rw::math::Q flim = _rdev->getMotorForceLimits();
        if( MetricUtil::dist1(flim,_maxForce)>0.0001 ){
            fmaxChanged = true;
            _maxForce = flim;
        }
    }
    //if(motormodesChanged)
    //	std::cout << "MOTOR MODE CHANGED: " << _rdev->getKinematicModel()->getName()<< "\n";

	_lastQ = _rdev->getModel().getQ(state);

	rw::math::Q targets = _rdev->getMotorTargets(state);
    //rw::math::Q velQ = _rdev->getVelocity(state);
    //rw::math::Q accLim = _rdev->getModel().getAccelerationLimits();
    //std::cout << velQ << "\n";

    int qi=0;
    for(size_t i = 0; i<_odeJoints.size(); i++){
        // dependend joints need to be handled separately
        bool depends = _odeJoints[i]->isDepend();

        if(depends){
            continue;
        }
        //TODO: make sure to stay within the actual acceleration limits
        //double vel = velQ(qi);
        //double avel = _odeJoints[i]->getActualVelocity();
        //double acc = (vel-avel)/dt;
        //std::cout << avel << ",";
        //if( fabs(acc)>accLim(qi) )
        //	acc = sign(acc)*accLim(qi);
        //vel = acc*dt+avel;
        //std::cout << accLim(qi) << ",";

        if(fmaxChanged)
            _odeJoints[i]->setMaxForce( _maxForce(qi) );

        if( _modes[qi]==RigidDevice::Velocity ){
            if(motormodesChanged)
                _odeJoints[i]->setMotorEnabled(true);
            _odeJoints[i]->setVelocity( targets[qi] );
        } else {
            // mode is force mode
            if(motormodesChanged)
                _odeJoints[i]->setMotorEnabled(false);
            _odeJoints[i]->setForce( targets[qi] );
            //std::cout << targets[qi] << " ; ";
        }
        qi++;
    }
    //std::cout  << " \n ";

    // we now handle the dependent joints
    for(size_t i = 0; i<_odeJoints.size(); i++){

        bool depends = _odeJoints[i]->isDepend();

        // dependend joints need to be handled separately
        if(depends){
            double oa = _odeJoints[i]->getOwner()->getAngle();
            double ov = _odeJoints[i]->getOwner()->getVelocity();
            double s = _odeJoints[i]->getScale();
            double off = _odeJoints[i]->getOffset();


            //double v = _odeJoints[i]->getVelocity();
            double a = _odeJoints[i]->getAngle();

            // the dependent joint need to be controlled such that the position/angle
            // in the next timestep will match that of the other joint

            // so first we look at the current position error. This should be
            // cancelled by adding a velocity

            double aerr  = (oa*s+off)-a;
            double averr = aerr/dt; // velocity that will cancel the error

            RW_ASSERT(_odeJoints[i]);
            rw::models::DependentPrismaticJoint* depJoint = NULL;
            if(_odeJoints[i]->getJoint()!=NULL)
                depJoint = dynamic_cast<rw::models::DependentPrismaticJoint*>(_odeJoints[i]->getJoint());

            if( depJoint!=NULL ){
                // specific PG70 solution

                //double aerr_pg70 = (oa*s+off)*2-a;
                _odeJoints[i]->setVelocity( 2*ov*s /*+ aerr_pg70/dt*/ );

                double oaerr_n  = ((a/2)-off)/s-oa;
                _odeJoints[i]->getOwner()->setVelocity( 0.5*oaerr_n/dt );

                //std::cout << "Setting velocity og PG70 : " << 2*ov*s << std::endl;
                //std::cout << "Setting velocity og PG70 owner: " << 0.5*oaerr_n/dt << std::endl;
                //std::cout << "dt " << dt << std::endl;
                //std::cout << "s " << s << std::endl;

                //_odeJoints[i]->getOwner()->setVelocity( 2*aerr_n/dt );
                //_odeJoints[i]->getOwner()->setVelocity( 0 );
                //_odeJoints[i]->setVelocity( /*2*ov*s +*/ aerr_pg70/dt );
                //std::cout << oa << " " << a << " " << aerr_pg70 << " " << averr << std::endl;
            } else {

                //_odeJoints[i]->setAngle(oa*s+off);
                //double averr = ov*s;
                // now we add the velocity that we expect the joint to have
                //averr += ov*s;

                // general solution
                //_odeJoints[i]->getOwner()->setVelocity(ov-0.1*averr/s);
                _odeJoints[i]->setVelocity(ov*s+averr);
            }
        }
    }


}

void ODEVelocityDevice::postUpdate(rw::kinematics::State& state){
    rw::math::Q velQ = _rdev->getVelocity(state);
    rw::math::Q q = _rdev->getModel().getQ(state);
    rw::math::Q actualVel = velQ;
    int qi = 0;
    for(size_t i = 0; i<_odeJoints.size(); i++){
        if(_odeJoints[i]->isDepend()){
            continue;
        }
        actualVel(qi) = _odeJoints[i]->getActualVelocity();

        /*if( !equalSign(_odeJoints[i]->getVelocity(),velQ(qi) ) ){
            double diff = fabs( velQ(qi) );
            if( diff>0.00001 ){
                //diff = std::max(0.1,diff)*10;
                _odeJoints[i]->setMaxForce( _maxForce(qi)*1.5);
            }
        } else {
            _odeJoints[i]->setMaxForce( _maxForce(qi) );
        }*/

        q(qi) = _odeJoints[i]->getAngle();
        qi++;
    }
    //std::cout  << "q without offset: " << q << std::endl;
    //std::cout  << "Actual vel: " << actualVel << std::endl;
    _rdev->getModel().setQ(q, state);
    _rdev->setJointVelocities(actualVel, state);

    //Q currentQ = _rdev->getModel().getQ(state);
    //Q myActVel = (_lastQ-currentQ)/_lastDt;
    //std::cout << "ActVel1: " << actualVel << std::endl;
    //std::cout << "ActVel2: " << myActVel << std::endl;
}



ODEVelocityDevice::ODEVelocityDevice(
            ODEBody *base,
            RigidDevice *rdev,
            const rw::kinematics::State& state,
            ODESimulator *sim
        ):
        _rdev(rdev),
        _modes(rdev->getMotorForceLimits().size(), RigidDevice::Velocity),
        _sim(sim)
{

     // we use hashspace here because devices typically have
     // relatively few bodies
     dSpaceID space = dHashSpaceCreate( sim->getODESpace() );

     _maxForce = rdev->getMotorForceLimits();


/*
     // add kinematic constraints from base to joint1, joint1 to joint2 and so forth
     Body *baseBody = _rdev->getBase();
     Frame *base = baseBody->getBodyFrame();

     // Check if the parent of base has been added to the ODE world,
     // if not create a fixed body whereto the base can be attached

     Frame *baseParent = base->getParent();
     dBodyID baseParentBodyID = 0;
     if(_rwFrameToODEBody.find(baseParent) == _rwFrameToODEBody.end()){
         //RW_WARN("No parent data available, connecting base to world!");
         dBodyID baseParentBodyId = dBodyCreate(_worldId);
         ODEUtil::setODEBodyT3D(baseParentBodyId, Kinematics::worldTframe(baseParent,state));
         baseParentBodyID = 0;
         //_rwFrameToODEBody[baseParent] = baseParentBodyId;
         _rwFrameToODEBody[baseParent] = 0;
     } else {
         baseParentBodyID = _rwFrameToODEBody[baseParent]->getBodyID();
     }

     // now create the base
     ODEBody *baseODEBody = createBody(baseBody, state, space);

     // and connect the base to the parent using a fixed joint if the base is rigid
     // we only do this if the parent is another body, NOT if its the world
     if( dynamic_cast<RigidBody*>(baseBody) ){
         if(_rwFrameToODEBody[baseParent]!=0){
             dJointID baseJoint = dJointCreateFixed(_worldId, 0);
             dJointAttach(baseJoint, baseBodyID, baseParentBodyID);
             dJointSetFixed(baseJoint);
             std::string name = _rwFrameToODEBody[baseParent]->getRwBody()->getBodyFrame()->getName();
             RW_DEBUGS("BASE: connecting base to body: "<<name);
         } else  {
             RW_DEBUGS("BASE: connecting base to world");

         }
     }
     */
     init(rdev, state, space, base);
}
namespace {
      void addToMap(ODEBody* b, std::map<Frame*, ODEBody*>& frameToODEBody){
          std::vector<Frame*> frames = b->getRwBody()->getFrames();
          BOOST_FOREACH(Frame* f, frames){
              frameToODEBody[f] = b;
          }
      }

      Joint* getParentJoint(Frame* f, const State& state){
          Joint* j = dynamic_cast<Joint*>(f);
          Frame* parent = f;
          while(j==NULL && parent!=NULL){
              j = dynamic_cast<Joint*>(parent);
              parent = f->getParent(state);
          }
          return j;
      }

}
void ODEVelocityDevice::init(RigidDevice *rdev, const rw::kinematics::State &state, dSpaceID spaceId, ODEBody* baseODEBody)
{
     //dBodyID baseBodyID = baseODEBody->getBodyID();
     std::map<Frame*, ODEBody*> frameToODEBody;

     addToMap(baseODEBody, frameToODEBody);

     std::map< Joint*, Body::Ptr> jointChildMap;
     std::map< Joint*, Body::Ptr> jointParentMap;

     std::map< Body::Ptr, Body::Ptr> childToParentMap;
     std::vector< std::pair<Body::Ptr, Body::Ptr> > childToParentList;
     baseODEBody->setTransform(state);
     // first we create rigid bodies from all of the links of the RigidDevice
     BOOST_FOREACH(Body::Ptr body, rdev->getLinks()){
         //std::cout << "LINK: " << body->getName() << std::endl;
         ODEBody *odebody = ODEBody::makeRigidBody(body, spaceId, _sim);
         odebody->setTransform( state );
         //_sim->addODEBody(odebody);
         _ode_bodies.push_back(odebody);
         addToMap(odebody, frameToODEBody);
         if(body->getBodyFrame()==rdev->getKinematicModel()->getBase()){
             childToParentMap[body] = NULL; // base
             continue;
         }
         // locate the parent body
         Body::Ptr jparent = BodyUtil::getParentBody(body, _sim->getDynamicWorkCell(), state );
         if(jparent==NULL)
             RW_THROW("The body \"" << body->getName() << "\" does not seem to have any parent body!");
         childToParentMap[body] = jparent; // base
         childToParentList.push_back(std::make_pair(body,jparent));

         /*
         // locate any parent joints and any child joints
         Joint *j = getParentJoint(body->getBodyFrame(), state);
         jointChildMap[j] = body;

         Body *jparent = BodyUtil::getParentBody(j, _sim->getDynamicWorkCell(), state );
         if(jparent==NULL)
             std::cout << "Parent is NULL" << std::endl;
         else
             std::cout << "Parent: " << jparent->getName() << std::endl;
         jointParentMap[j] = jparent;
         */
     }

     std::map<Joint*,ODEJoint*> jointMap;
     // now locate all joints connecting the child-parent body pairs
     typedef std::pair<Body::Ptr,Body::Ptr> BodyPair;
     BOOST_FOREACH( BodyPair bodyPair, childToParentList){
         std::vector<Frame*> chain = Kinematics::parentToChildChain(bodyPair.second->getBodyFrame(), bodyPair.first->getBodyFrame(), state);
         chain.push_back(bodyPair.first->getBodyFrame());
         std::vector<Joint*> joints;
         for(int i=1;i<(int)chain.size();i++){
             Joint *joint= dynamic_cast<Joint*>(chain[i]);
             if( joint!=NULL){
                 // connect this joint to the parent body and if
                 joints.push_back(joint);
             }
         }

         ODEBody *parent = frameToODEBody[bodyPair.second->getBodyFrame()];
         for(int i=0;i<(int)joints.size();i++){
             // the child of the joint is either bodyPair.first or another joint in which case we need to make an ODEBody
             ODEBody *child = frameToODEBody[bodyPair.first->getBodyFrame()];

             if(child==NULL){
            	 // in case not bodies are placed between two joints then add a virtual body...
            	 // TODO: this is somewhat of a hack please remove/change/fix
            	 // the user should specify links in the DWC
            	 std::cout << "creating virtual body" << std::endl;
                 dBodyID bTmp = dBodyCreate( _sim->getODEWorldId() );
                 ODEUtil::setODEBodyMass(bTmp,0.01, Vector3D<>(0,0,0), InertiaMatrix<>::makeSolidSphereInertia(0.01,1) );
                 child = new ODEBody(bTmp, joints[i]);
                 child->setTransform( state );
                 _sim->addODEBody(child);
             }

             /*
             if(i+1 == (int)joints.size()){
                 // this is the last joint so connect it to bodyPair.first
            	 child = frameToODEBody[bodyPair.first->getBodyFrame()];
             }
             else {
                 // create a virtual body
            	 std::cout << "creating virtual body" << std::endl;
                 dBodyID bTmp = dBodyCreate( _sim->getODEWorldId() );
                 ODEUtil::setODEBodyMass(bTmp,0.01, Vector3D<>(0,0,0), InertiaMatrix<>::makeSolidSphereInertia(0.01,1) );
                 child = new ODEBody(bTmp, joints[i]);
                 child->setTransform( state );
                 _sim->addODEBody(child);
             }
             */
             ODEJoint *odeJoint = new ODEJoint(joints[i], parent, child, _sim, state);
             _sim->addODEJoint(odeJoint);
             jointMap[joints[i]] = odeJoint;

         }
     }

     // in the end we fix the force limits
     std::vector<ODEJoint*> odeJoints;
     Q maxForce = rdev->getMotorForceLimits();
     size_t i =0;
     rw::models::JointDevice::Ptr jdev = rdev->getJointDevice();
     BOOST_FOREACH(Joint *joint, jdev->getJoints() ){
         ODEJoint *odeJoint = jointMap[joint];
         RW_ASSERT(odeJoint!=NULL);
         _odeJoints.push_back(odeJoint);
         odeJoint->setMaxForce( maxForce(i) );
         i++;
     }

     //RW_DEBUGS("BASE:" << base->getName() << "<--" << base->getParent()->getName() );

     // build a parent-child relation map between joints and links
     /*
     size_t i =0;
     rw::models::JointDevice::Ptr jdev = rdev->getJointDevice();
     BOOST_FOREACH(Joint *joint, jdev->getJoints() ){
         Body *parentb = jointParentMap[joint];
         Body *childb = jointChildMap[joint];
         Frame *parent = parentb->getBodyFrame();
         Frame *child = childb->getBodyFrame();
         // find the body or joint belonging to parent
         //std::cout << parent->getName() << "<--" << joint->getName() << std::endl;

         ODEBody *odeParent = frameToODEBody[parent];
         ODEBody *odeChild = frameToODEBody[child];

         if(odeParent==NULL ){
             RW_THROW("ODEParent is NULL, " << child->getName() << "-->"<< parent->getName());
         }

         if(odeChild==NULL ){
             RW_THROW("ODEChild is NULL, " << child->getName() << "-->" << parent->getName());
         }

         //Transform3D<> wTchild = Kinematics::worldTframe(child,state);
         //Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
         //Vector3D<> hpos = wTchild.P();
         //Transform3D<> wTparent = Kinematics::WorldTframe(parentFrame,initState);

         std::pair<Q, Q> posBounds = joint->getBounds();

         ODEJoint *odeJoint = new ODEJoint(joint, odeParent, odeChild, _sim, state);
         _sim->addODEJoint(odeJoint);

         odeJoint->setMaxForce( maxForce(i) );

         _odeJoints.push_back( odeJoint );

          i++;
     }
    */

}






/*
ODEVelocityDevice* ODEVelocityDevice::makeDevice(RigidDevice *rdev,
												 dBodyID base,
												 dSpaceID space,
												 dWorldID worldId){
    JointDevice *jdev = dynamic_cast<JointDevice*>( &(rDev->getModel()) );
    Q offsets = Q::zero( jdev->getQ(state).size() );
    jdev->setQ( offsets , initState );

    std::vector<ODEJoint*> odeJoints;
    Q maxForce = rDev->getForceLimit();
    // std::cout  << "Max force: " << maxForce << std::endl;
    // std::cout  << "Nr of Joints " << jdev->getDOF()<< std::endl;
    Frame *rwBase = rDev->getModel().getBase();

    size_t i =0;
    BOOST_FOREACH(RigidJoint *rjoint, rDev->getBodies() ){
        Frame *joint = &rjoint->getFrame();
        Frame *parent = joint->getParent(initState);
        //std::cout  << parent->getName() << "-->" << joint->getName() << std::endl;

        dBodyID odeParent = _rwFrameToODEBody[parent];
        Frame *parentFrame = NULL;
        if(odeParent==NULL){
            // could be that the reference is
            RW_WARN("odeParent is NULL, " << joint->getName() << "-->"
					<< parent->getName()
					<< " Using WORLD as parent");
            odeParent = _rwFrameToODEBody[wframe];
        } else {
            parentFrame = _rwODEBodyToFrame[odeParent];
        }

        dBodyID odeChild = createRigidBody(rjoint, rjoint->getInfo() , initState, space ); //_rwFrameToBtBody[joint];
        if(odeChild==NULL){
            RW_WARN("odeChild is NULL, " << joint->getName());
            RW_ASSERT(0);
        }

        //std::cout  << parent->getName() << " " << joint->getName() << std::endl;
        Transform3D<> wTchild = Kinematics::worldTframe(joint,initState);
        Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
        Vector3D<> hpos = wTchild.P();

        //Transform3D<> wTparent = Kinematics::WorldTframe(parentFrame,initState);

         if(RevoluteJoint *rwjoint = dynamic_cast<RevoluteJoint*>(joint)){
             const double qinit = rwjoint->getQ(initState)[0];

             dJointID hinge = dJointCreateHinge (_worldId, 0);
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
         } else if( dynamic_cast<PrismaticJoint*>(joint) ){
             // std::cout  << "Slider constraint" << std::endl;

             //_jointToConstraintMap[joint] = slider;
             //constraints.push_back(slider);
         //} else if(  ) {

         } else if( dynamic_cast<PassiveRevoluteFrame*>(joint)){
             PassiveRevoluteFrame *rframe = dynamic_cast<PassiveRevoluteFrame*>(joint);
             Joint *owner = &rframe->getOwner();
             const double qinit = owner->getQ(initState)[0]*rframe->getScale()+0;

             dJointID hinge = dJointCreateHinge (_worldId, 0);
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

             //std::cout << "CREATED ODEJOINT: PAS" << joint->getName() << std::endl;

             ODEJoint *odeOwner = _jointToODEJoint[owner];
             ODEJoint *odeJoint = new ODEJoint( hinge, motor, odeOwner, rframe->getScale(),0 );
             odeJoints.push_back(odeJoint);
             //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
             //dJointSetAMotorParam(Amotor,dParamHiStop,0);

         } else {
             RW_WARN("Joint type not supported!");
         }

         i++;
      }

}
*/
