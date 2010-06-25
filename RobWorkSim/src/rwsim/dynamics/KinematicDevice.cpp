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

#include <rw/common/macros.hpp>

using namespace rwsim::dynamics;

KinematicDevice::KinematicDevice(
				dynamics::Body* base,
				const std::vector<dynamics::KinematicBody*> bodies,
                rw::models::Device *dev,
                rw::models::WorkCell* wc):
                    DynamicDevice(base,dev,wc),
                    _bodies(bodies),
                    _maxVel(dev->getVelocityLimits()),
                    _maxAcc(dev->getAccelerationLimits()),
                    _q( rw::math::Q::zero(dev->getDOF()) ),
                    _velQ( rw::math::Q::zero(dev->getDOF()) )
{

}

KinematicDevice::~KinematicDevice(){

}

const std::vector<KinematicBody*>&
    KinematicDevice::getBodies(){

    return _bodies;
}


// parameters for velocity profile
void KinematicDevice::setMaxAcc(const rw::math::Q& acc){
    RW_ASSERT( acc.size()==_dev->getDOF() );
    // todo
    _maxAcc = acc;
}

rw::math::Q KinematicDevice::getMaxAcc(){
    return _maxAcc;
}

void KinematicDevice::setMaxVel(const rw::math::Q& vel){
    RW_ASSERT( vel.size()== _dev->getDOF() );
    // todo
    _maxVel = vel;
}

rw::math::Q KinematicDevice::getMaxVel(){
    return _maxVel;
}
