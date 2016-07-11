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

#include "RigidDevice.hpp"

//#include <rw/models/RevoluteJoint.hpp>
//#include <rw/models/PrismaticJoint.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Joint.hpp>

using namespace rwsim::dynamics;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;

namespace {

    template<class T>
    T* findParentFrom(rw::kinematics::Frame* f){
        Frame* parent = f;
        while(parent!=NULL){
            T* res = dynamic_cast<T*>(parent);
            if(res!=NULL)
                return res;
            parent = parent->getParent();
        }
        return NULL;
    }

    class RigidLink : public Body
    {
    public:
        RigidLink( const BodyInfo& info, rw::models::Object::Ptr obj, RigidDevice *ddev, size_t id):
            Body(info,obj),_ddev(ddev),_id(id)
        {
        	add(_rstate);
            // find the joint index for which this link is attached
            Joint *firstParentJoint = findParentFrom<Joint>(obj->getBase());
            _jointFrame = firstParentJoint;
            // check which index this joint has in the device
            int idx=0;
            BOOST_FOREACH(Joint* j, _ddev->getJointDevice()->getJoints()){
                if(firstParentJoint==j){
                    _jointIdx = idx;
                    break;
                }
                idx++;
            }
        }

        virtual ~RigidLink(){}

    public: // functions that need to be implemented by specialized class

        //! @copydoc Body::getPointVelW
        virtual rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const{
            Transform3D<> bTf = _ddev->getModel().baseTframe(getBodyFrame(), state);
            Q vel = _ddev->getJointVelocities(state);
            Jacobian bJf = _ddev->getModel().baseJframe(getBodyFrame(), state);
            return inverse(bTf) * (bJf*vel.getSubPart(0, _jointIdx+1));
        }

         virtual void reset(rw::kinematics::State &state){}

         virtual double calcEnergy(const State& state,
         		const Vector3D<>& gravity = Vector3D<>::zero(),
 				const Vector3D<>& potZero = Vector3D<>::zero()) const {
             return 0;
         }


         //! @copydoc Body::setForce
         void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
             _rstate.get(state).force = f;
         }

         //! @copydoc Body::addForce
         void addForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
             _rstate.get(state).force += f;
         }

         //! @copydoc Body::getForce
         rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const {
             return _rstate.get(state).force;
         }

         //! @copydoc Body::setTorque
         void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
             _rstate.get(state).torque = t;
         }

         //! @copydoc Body::addTorque
         void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
             _rstate.get(state).torque += t;
         }

         //! @copydoc Body::getTorque
         rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
             return _rstate.get(state).torque;
         }

         RigidDevice* getDynamicDevice(){ return _ddev; }

         size_t getID(){ return _id; }
    private:
         rw::models::Object::Ptr _obj;
        RigidDevice *_ddev;
        size_t _id;
        int _jointIdx;
        Joint *_jointFrame;
        struct RigidLinkState {
            rw::math::Vector3D<> force;
            rw::math::Vector3D<> torque;
        };

        rw::kinematics::StatelessData<RigidLinkState> _rstate;
    };


}


RigidDevice::RigidDevice(rwsim::dynamics::Body::Ptr base,
                         const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
                         rw::models::JointDevice::Ptr dev):
     DynamicDevice(base,dev),
     _velocity((int)dev->getDOF()),
     _target((int)dev->getDOF()),
     _mode((int)dev->getDOF()),
     //_torque(this,dev->getDOF()),
     //_vel( rw::math::Q::zero(dev->getDOF()) ),
     //_actualVel( rw::math::Q::zero(dev->getDOF()) ),
    _forceLimits( rw::math::Q::zero(dev->getDOF()) ),
    _jdev(dev)
{
    for(size_t i=0;i<objects.size(); i++){
        _links.push_back( rw::common::ownedPtr( new RigidLink(objects[i].first, objects[i].second, this, i) ) );
        //this->add( *_links.back() );
    }

	add( _velocity );
	add( _target );
    add( _mode );

}


void RigidDevice::setMotorForceLimits(const rw::math::Q& force){
    _forceLimits = force;
}

rw::math::Q RigidDevice::getMotorForceLimits(){
    return _forceLimits;
}

rw::math::Q RigidDevice::getJointVelocities(const rw::kinematics::State& state){
    const double *vals = _velocity.getArray(state);
    return rw::math::Q(_velocity.getN(), vals);
}

double RigidDevice::getJointVelocity(int i, const rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_velocity.getN());
    return _velocity.getArray(state)[i];
}

std::vector<RigidDevice::MotorControlMode> RigidDevice::getMotorModes(const rw::kinematics::State& state){
    std::vector<RigidDevice::MotorControlMode> res(_mode.getN(), RigidDevice::Velocity);
    char *arr = _mode.getArray(state);
    for(int i=0;i<_mode.getN();i++){
        if(arr[i]==0){
            res[i] = RigidDevice::Velocity;
        } else {
            res[i] = RigidDevice::Force;
        }
    }
    return res;
}

RigidDevice::MotorControlMode RigidDevice::getMotorMode(int i, const rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_mode.getN());
    if(_mode.getArray(state)[i]==0){
        return RigidDevice::Velocity;
    }
    return RigidDevice::Force;
}

rw::math::Q RigidDevice::getMotorTargets(const rw::kinematics::State& state){
    const double *vals = _target.getArray(state);
    return rw::math::Q(_target.getN(), vals);
}

double RigidDevice::getMotorTarget(int i, const rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_target.getN());
    return _target.getArray(state)[i];
}

void RigidDevice::setMotorTargets(const rw::math::Q& q, rw::kinematics::State& state){
    double *vals = _target.getArray(state);
    for(int i=0;i<std::min(_target.getN(),(int)q.size());i++){
        vals[i] = q[i];
    }
}

void RigidDevice::setMotorForceTargets(const rw::math::Q& q, rw::kinematics::State& state){
	//std::cout << "setMotorForceTargets: " << q << std::endl;
    double *vals = _target.getArray(state);
    char *modes = _mode.getArray(state);

    for(int i=0;i<std::min(_target.getN(),(int)q.size());i++){
        vals[i] = q[i];
        modes[i] = 1;
    }
}

void RigidDevice::setMotorVelocityTargets(const rw::math::Q& q, rw::kinematics::State& state){
    //std::cout << "setMotorVelocityTargets: " << q << std::endl;
	double *vals = _target.getArray(state);
    char *modes = _mode.getArray(state);

    rw::math::Q velLimit = _jdev->getVelocityLimits();

    for(int i=0;i<std::min(_target.getN(),(int)q.size());i++){
        vals[i] = Math::clamp(q[i], -velLimit[i], velLimit[i]);
        modes[i] = 0;
    }
}


void RigidDevice::setJointVelocities(const rw::math::Q& q, rw::kinematics::State& state){
    double *vals = _velocity.getArray(state);
    rw::math::Q velLimit = _jdev->getVelocityLimits();
    for(int i=0;i<std::min(_velocity.getN(),(int)q.size());i++){
        vals[i] = Math::clamp(q[i], -velLimit[i], velLimit[i]);
    }
}

void RigidDevice::setJointVelocity(double vel, int i, rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_velocity.getN());
    double *vals = _velocity.getArray(state);
    vals[i] = vel;
}


void RigidDevice::setMotorTarget(double q, int i, rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_target.getN());
    double *vals = _target.getArray(state);
    vals[i] = q;
}

void RigidDevice::setMotorForceTarget(double force, int i, rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_target.getN());
    double *vals = _target.getArray(state);
    char *modes = _mode.getArray(state);
    vals[i] = force;
    modes[i] = 1;
}

void RigidDevice::setMotorVelocityTarget(double vel, int i, rw::kinematics::State& state){
    RW_ASSERT(i>=0);
    RW_ASSERT(i<_target.getN());
    double *vals = _target.getArray(state);
    char *modes = _mode.getArray(state);
    vals[i] = vel;
    modes[i] = 0;
}

/*
void RigidDevice::registerIn(rw::kinematics::StateStructure::Ptr statestructure){
    DynamicDevice::registerIn(statestructure);

    // add all links to the register
    BOOST_FOREACH(Body::Ptr link, getLinks()){
        link->registerIn(statestructure);
    }

}
*/




















#ifdef wedfmdfmd

void RigidDevice::setVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
    rw::math::Q velLimit = getModel().getVelocityLimits();

    RW_ASSERT(vel.size()==velLimit.size());

   // std::cout  << "Vel limits: " << velLimit <<  std::endl;
   // std::cout  << "Before clamp: " << vel << std::endl;
    _vel = rw::math::Math::clampQ(vel, -velLimit, velLimit);
   // std::cout  << "after  clamp: " << _vel << std::endl;
}


void RigidDevice::addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state)
{
    _torque = forceTorque;
    _force = forceTorque;
    /*
    for(size_t i=0;i<_bodies.size(); i++){
        double ft = forceTorque(i);
        //_bodies[i]->addJointForce( ft, rw::kinematics::State& state);
        rw::models::Joint *joint = _bodies[i]->getJoint();
        if( dynamic_cast<rw::models::RevoluteJoint*>(joint) ){
            _bodies[i]->addTorque( Vector3D<>(0,0,ft), state );
        } else if( dynamic_cast<rw::models::PrismaticJoint*>(joint) ){
            _bodies[i]->addForce( Vector3D<>(0,0,ft), state );
        }
    }
    */
}

#endif

