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

#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/math/Vector3D.hpp>
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
            Body(6,info,obj),_ddev(ddev),_id(id)
        {
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
            // Todo: get joint velocity, and from that calculate the velocity of this body in
            // the joint frame
            return rw::math::VelocityScrew6D<>();
        }

         virtual void reset(rw::kinematics::State &state){

         }

         virtual double calcEnergy(const rw::kinematics::State& state){
             return 0;
         }


         //! @copydoc Body::setForce
         void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
             double *q = this->getData(state);
             q[0] = f[0];
             q[1] = f[1];
             q[2] = f[2];
         }

         //! @copydoc Body::addForce
         void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state){
             double *q = this->getData(state);
             q[0] += force[0];
             q[1] += force[1];
             q[2] += force[2];
         }

         //! @copydoc Body::getForce
         rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const {
             const double *q = this->getData(state);
             return rw::math::Vector3D<>(q[0],q[1],q[2]);
         }

         //! @copydoc Body::setTorque
         void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
             double *q = this->getData(state);
             q[3]  = t[0];
             q[4] = t[1];
             q[5] = t[2];
         }

         //! @copydoc Body::addTorque
         void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
             double *q = this->getData(state);
             q[3]  += t[0];
             q[4] += t[1];
             q[5] += t[2];
         }

         //! @copydoc Body::getTorque
         rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
             const double *q = this->getData(state);
             return rw::math::Vector3D<>(q[3],q[4],q[5]);
         }

         RigidDevice* getDynamicDevice(){ return _ddev; }

         size_t getID(){ return _id; }
    private:
         rw::models::Object::Ptr _obj;
        RigidDevice *_ddev;
        size_t _id;
        int _jointIdx;
        Joint *_jointFrame;
    };


}


RigidDevice::RigidDevice(dynamics::Body* base,
                         const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
                         rw::models::JointDevice::Ptr dev):
     DynamicDevice(base,dev),
    _vel( rw::math::Q::zero(dev->getDOF()) ),
    _actualVel( rw::math::Q::zero(dev->getDOF()) ),
    _force( rw::math::Q::zero(dev->getDOF()) ),
    _jdev(dev)
{
    for(size_t i=0;i<objects.size(); i++){
        _links.push_back( new RigidLink(objects[i].first, objects[i].second, this, i) );
    }
}


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
