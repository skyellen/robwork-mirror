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

#include "ODEJoint.hpp"
#include <ode/ode.h>

#include <rw/math/MetricUtil.hpp>

#include <boost/foreach.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

namespace {

    bool equalSign(double v1, double v2 ){
        return (v1<0 && v2<0) || (v1>0 && v2>0);
    }

}

ODEVelocityDevice::ODEVelocityDevice(
            RigidDevice *rdev,
            std::vector<ODEJoint*> odeJoints,
            Q maxForce):
        _rdev(rdev),
        _odeJoints(odeJoints),
        _maxForce(maxForce)
{

}

ODEVelocityDevice::~ODEVelocityDevice(){};

void ODEVelocityDevice::reset(rw::kinematics::State& state){
    rw::math::Q q = _rdev->getModel().getQ(state);
    rw::math::Q flim = _rdev->getForceLimit();
    int qi = 0;
    for(size_t i = 0; i<_odeJoints.size(); i++){
        _odeJoints[i]->setVelocity( 0 );
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            continue;
        }
        _odeJoints[i]->setAngle( q(qi) );
        _odeJoints[i]->setMaxForce( flim(qi) );
        qi++;
    }
    for(size_t i = 0; i<_odeJoints.size(); i++){
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
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


void ODEVelocityDevice::update(double dt, rw::kinematics::State& state){
	rw::math::Q flim = _rdev->getForceLimit();
	bool fmaxChanged = false;
	if( MetricUtil::dist2(flim,_maxForce)>0.0001 ){
		fmaxChanged = true;
		_maxForce = flim;
	}

    rw::math::Q velQ = _rdev->getVelocity(state);
    rw::math::Q accLim = _rdev->getModel().getAccelerationLimits();

    int qi=0;
    for(size_t i = 0; i<_odeJoints.size(); i++){
        // dependend joints need to be handled separately
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            continue;
        }
        //TODO: make sure to stay within the actual acceleration limits
        double vel = velQ(qi);

        double avel = _odeJoints[i]->getActualVelocity();
        double acc = (vel-avel)/dt;
        std::cout << avel << ",";
        if( fabs(acc)>accLim(qi) )
        	acc = sign(acc)*accLim(qi);
        vel = acc*dt+avel;
        std::cout << accLim(qi) << ",";


        _odeJoints[i]->setVelocity( vel );
        if(fmaxChanged)
        	_odeJoints[i]->setMaxForce( _maxForce(qi) );

        qi++;
    }
    std::cout << std::endl;

    // we now handle the dependent joints
    for(size_t i = 0; i<_odeJoints.size(); i++){
        // dependend joints need to be handled separately
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
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
            double averr = 0.5*aerr/dt; // velocity that will cancel the error
            // now we add the velocity that we expect the joint to have
            //averr += ov*s;

            RW_ASSERT(_odeJoints[i]);
            RW_ASSERT(_odeJoints[i]->getRigidJoint());
            //RW_ASSERT(_odeJoints[i]->getRigidJoint()->getJoint());
            rw::models::DependentPrismaticJoint* depJoint = NULL;
            std::cout << "a " << std::endl;
            if(_odeJoints[i]->getRigidJoint()->getJoint()!=NULL)
                depJoint = dynamic_cast<rw::models::DependentPrismaticJoint*>(_odeJoints[i]->getRigidJoint()->getJoint());
            std::cout << "a " << std::endl;
            if( depJoint!=NULL ){
                std::cout << "3 " << std::endl;
                // specific PG70 solution

                double aerr_n  = ((a/2)/s-off)-oa;
                _odeJoints[i]->getOwner()->setVelocity( aerr_n/dt );
                _odeJoints[i]->setVelocity(ov*s);
                std::cout << "setVel: " << ov*s << std::endl;
            } else {
                std::cout << "4 " << std::endl;
                //_odeJoints[i]->setAngle(oa*s+off);
                double averr = ov*s;


                // general solution
                _odeJoints[i]->getOwner()->setVelocity(ov-averr/s);
                _odeJoints[i]->setVelocity(averr);
            }
        }
    }
    std::cout << "END UPDATE " << std::endl;
}

void ODEVelocityDevice::postUpdate(rw::kinematics::State& state){
    rw::math::Q velQ = _rdev->getVelocity(state);
    rw::math::Q q = _rdev->getModel().getQ(state);
    rw::math::Q actualVel = velQ;
    int qi = 0;
    for(size_t i = 0; i<_odeJoints.size(); i++){

        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
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
    _rdev->setActualVelocity(actualVel, state);
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
    Q maxForce = fDev->getForceLimit();
    // std::cout  << "Max force: " << maxForce << std::endl;
    // std::cout  << "Nr of Joints " << jdev->getDOF()<< std::endl;
    Frame *rwBase = rDev->getModel().getBase();

    size_t i =0;
    BOOST_FOREACH(RigidJoint *rjoint, fDev->getBodies() ){
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
