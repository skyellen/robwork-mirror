/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtPositionDevice.hpp"
#include "BtUtil.hpp"

#include <rwsim/dynamics/KinematicDevice.hpp>

#include <BulletDynamics/Dynamics/btRigidBody.h>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::bullet;

BtPositionDevice::BtPositionDevice(rw::common::Ptr<KinematicDevice> dev, const std::vector<FrameBodyPair>& frameToBtBody):
	_kdev(dev),
	_frameToBtBody(frameToBtBody)
{
}

BtPositionDevice::~BtPositionDevice() {
}

void BtPositionDevice::update(double dt, State& state){
    _kdev->getModel().setQ( _kdev->getQ(state), state);
    // for each joint update the position of the corresponding btRigidBody

    BOOST_FOREACH(const FrameBodyPair& pair, _frameToBtBody ){
        const Transform3D<> t3d = Kinematics::worldTframe( pair.first, state);
        pair.second->getMotionState()->setWorldTransform( BtUtil::makeBtTransform(t3d) );
    }
}

void BtPositionDevice::postUpdate(State& state) {
}
