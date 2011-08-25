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
