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

#include "KinematicDevice.hpp"
#include <rw/models/Joint.hpp>

#include <rw/common/macros.hpp>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsim;

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

    class KinematicLink : public Body
    {
    public:
        KinematicLink( const BodyInfo& info, rw::models::Object::Ptr obj, KinematicDevice *ddev, size_t id):
            Body(info,obj),_ddev(ddev),_id(id)
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

        virtual ~KinematicLink(){}

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
         void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){}

         //! @copydoc Body::addForce
         void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state){}

         //! @copydoc Body::getForce
         rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const {
             return Vector3D<>(0,0,0);
         }

         //! @copydoc Body::setTorque
         void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){}

         //! @copydoc Body::addTorque
         void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){}

         //! @copydoc Body::getTorque
         rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
             return Vector3D<>(0,0,0);
         }


         KinematicDevice* getDynamicDevice(){ return _ddev; }

         size_t getID(){ return _id; }
    private:
         rw::models::Object::Ptr _obj;
         KinematicDevice *_ddev;
        size_t _id;
        int _jointIdx;
        Joint *_jointFrame;

    };


}

KinematicDevice::KinematicDevice(
				dynamics::Body::Ptr base,
				const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
                rw::models::JointDevice::Ptr dev):
                    DynamicDevice(base,dev),
                    _maxVel(dev->getVelocityLimits()),
                    _maxAcc(dev->getAccelerationLimits()),
                    _jdev(dev),
                    _velocity((int)dev->getDOF())
{
    for(size_t i=0;i<objects.size(); i++){
        _links.push_back( rw::common::ownedPtr( new KinematicLink(objects[i].first, objects[i].second, this, i) ) );
    }
    add(_velocity);
}

KinematicDevice::~KinematicDevice(){
}


// parameters for velocity profile
void KinematicDevice::setMaxAcc(const rw::math::Q& acc){
    RW_ASSERT( acc.size()==_dev->getDOF() );
    _maxAcc = acc;
}

rw::math::Q KinematicDevice::getMaxAcc(){
    return _maxAcc;
}

void KinematicDevice::setMaxVel(const rw::math::Q& vel){
    RW_ASSERT( vel.size()== _dev->getDOF() );
    _maxVel = vel;
}

rw::math::Q KinematicDevice::getMaxVel(){
    return _maxVel;
}
