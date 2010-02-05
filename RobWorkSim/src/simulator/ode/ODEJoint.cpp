/*
 * ODEJoint.cpp
 *
 *  Created on: 10-09-2008
 *      Author: jimali
 */

#include "ODEJoint.hpp"
#include "ODEUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>

#include <ode/ode.h>

using namespace rw::kinematics;
using namespace rw::math;


ODEJoint::ODEJoint(
         dJointID odeJoint,
         dJointID odeMotor,
         dBodyID body,
         dynamics::RigidJoint* rwjoint):
             _jointId(odeJoint),
             _bodyId(body),
             _motorId(odeMotor),
             _rwJoint(rwjoint),
             _owner(NULL),
             _type(ODEJoint::RIGID),
             _bodyFrame(&rwjoint->getBodyFrame()),
             _offset( rwjoint->getInfo().masscenter )
{

}

ODEJoint::ODEJoint(dJointID odeJoint,
         dJointID odeMotor,
         dBodyID body,
         ODEJoint* owner,
         rw::kinematics::Frame *bframe,
         double scale,
         double off):
             _jointId(odeJoint),
             _motorId(odeMotor),
             _bodyId(body),
             _owner(owner),
             _scale(scale),
             _off(off),
             _type(ODEJoint::DEPEND),
             _bodyFrame(bframe),
             _offset(0,0,0)
{

}
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
    if(_type!=ODEJoint::DEPEND){
        Frame *bframe = &_rwJoint->getFrame();

        Transform3D<> wTb = rw::kinematics::Kinematics::worldTframe( bframe, state);
        wTb.P() += wTb.R()*_offset;
        ODEUtil::setODEBodyT3D( _bodyId, wTb );
    }

    dBodyEnable( _bodyId );
    dBodySetAngularVel( _bodyId, 0, 0, 0 );
    dBodySetLinearVel( _bodyId, 0, 0, 0 );
}

