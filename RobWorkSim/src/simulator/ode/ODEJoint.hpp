/*
 * ODEBody.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEJOINT_HPP_
#define ODEJOINT_HPP_

#include <ode/ode.h>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <dynamics/Body.hpp>
#include <dynamics/RigidJoint.hpp>
#include <rw/math/Vector3D.hpp>

/**
 * @brief this class bridges ODE's joints with RobWork joints.
 */
class ODEJoint {
public:
    typedef enum{FIXED, RIGID, DEPEND} ODEJointType;

    /**
     * @brief constructor
     * @param odeJoint
     * @param odeMotor
     * @param rwbody
     * @return
     */
    ODEJoint(
             dJointID odeJoint,
             dJointID odeMotor,
             dBodyID body,
             dynamics::RigidJoint* rwbody);

    ODEJoint(dJointID odeJoint,
             dJointID odeMotor,
             dBodyID body,
             ODEJoint* owner,
             rw::kinematics::Frame *bframe,
             double scale, double off);

    virtual ~ODEJoint(){};

    void setVelocity(double vel){
        dJointSetAMotorParam(_motorId, dParamVel, vel);
    }

    double getVelocity(){
        return dJointGetAMotorParam(_motorId, dParamVel);
    }

    void setAngle(double pos){
        dJointSetAMotorAngle(_motorId, 0, pos);
    }

    double getAngle(){
        return dJointGetHingeAngle( _jointId );
        //return dJointGetAMotorAngle(_motorId, 0);
    }

    double getActualVelocity(){
        return dJointGetHingeAngleRate( _jointId );
    }

    void setMaxForce(double force){
        dJointSetAMotorParam(_motorId,dParamFMax, force );
    }

    double getMaxForce(){
        return dJointGetAMotorParam(_motorId,dParamFMax);
    }

    ODEJoint* getOwner(){
        return _owner;
    }

    dBodyID getODEBody(){
        return _bodyId;
    }

    ODEJointType getType(){
        return _type;
    }

    void reset(const rw::kinematics::State& state);

    double getScale(){ return _scale; };
    double getOffset(){ return _off; };

    //static ODEJoint* make(RevoluteJoint* joint, dBodyID parent);

private:
    dBodyID _bodyId;
    dJointID _jointId, _motorId;
    dynamics::RigidJoint *_rwJoint;

    ODEJoint *_owner;
    double _scale,_off;
    ODEJointType _type;

    rw::kinematics::Frame *_bodyFrame;
    rw::math::Vector3D<> _offset;
};


#endif /* ODEBODY_HPP_ */
