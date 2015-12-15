/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rwsim/dynamics/FixedBody.hpp>
#include "RWPEBodyFixed.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEBodyFixed::RWPEBodyFixed(FixedBody::Ptr body):
	RWPEBody(body)
{
}

RWPEBodyFixed::~RWPEBodyFixed() {
}

FixedBody::Ptr RWPEBodyFixed::getFixedBody() const {
	return get().cast<FixedBody>();
}

void RWPEBodyFixed::updateRW(State &rwstate, const RWPEIslandState &islandState) const {
	// Nothing to update in RW
}

VelocityScrew6D<> RWPEBodyFixed::getVelocityW(const State &rwstate, const RWPEIslandState &islandState) const {
	return VelocityScrew6D<>();
}
