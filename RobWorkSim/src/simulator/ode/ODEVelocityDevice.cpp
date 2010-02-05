/*
 * ODEVelocityDevice.cpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */
#include "ODEVelocityDevice.hpp"

#include "ODEJoint.hpp"
#include <ode/ode.h>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace dynamics;

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
    int qi = 0;
    for(int i = 0; i<_odeJoints.size(); i++){
        _odeJoints[i]->setVelocity( 0 );
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            continue;
        }
        _odeJoints[i]->setAngle( q(qi) );
        qi++;
    }
    for(int i = 0; i<_odeJoints.size(); i++){
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            double angle = _odeJoints[i]->getOwner()->getAngle();
            _odeJoints[i]->setAngle(angle);
        }
        _odeJoints[i]->reset(state);
    }
}


void ODEVelocityDevice::update(double dt, rw::kinematics::State& state){
    rw::math::Q velQ = _rdev->getVelocity(state);
    int qi=0;
    for(size_t i = 0; i<_odeJoints.size(); i++){
        // dependend joints need to be handled separately
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            continue;
        }
        _odeJoints[i]->setVelocity( velQ(qi) );
        qi++;
    }

    // we now handle the dependent joints
    for(size_t i = 0; i<_odeJoints.size(); i++){
        // dependend joints need to be handled separately
        if(_odeJoints[i]->getType()==ODEJoint::DEPEND){
            double oa = _odeJoints[i]->getOwner()->getAngle();
            double ov = _odeJoints[i]->getOwner()->getVelocity();
            double s = _odeJoints[i]->getScale();
            double off = _odeJoints[i]->getOffset();

            /*          double v = _odeJoints[i]->getVelocity();
            double a = _odeJoints[i]->getAngle();

            if( fabs(v)<fabs(ov) ){
                ov = (a + v*dt - s*oa - off)/dt;
                _odeJoints[i]->getOwner()->setVelocity
            }

*/          std::cout << "Owner angle: " << oa << " " << s << std::endl;
            //_odeJoints[i]->setAngle(oa*s+off);
            _odeJoints[i]->setVelocity(ov*s);
        }
    }
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
        actualVel(i) = _odeJoints[i]->getActualVelocity();

        if( !equalSign(_odeJoints[i]->getVelocity(),velQ(qi) ) ){
            double diff = fabs( velQ(qi) );
            if( diff>0.00001 ){
                //diff = std::max(0.1,diff)*10;
                _odeJoints[i]->setMaxForce( _maxForce(i)*2);
            }
        } else {
            _odeJoints[i]->setMaxForce( _maxForce(i) );
        }

        q(qi) = _odeJoints[i]->getAngle();
        qi++;
    }
    //std::cout  << "q without offset: " << q << std::endl;
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
