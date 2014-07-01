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

#include "TNTFixedBody.hpp"

#include <rwsim/dynamics/FixedBody.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTFixedBody::TNTFixedBody(FixedBody::Ptr body):
	TNTBody(body)
{
}

TNTFixedBody::~TNTFixedBody() {
}

FixedBody::Ptr TNTFixedBody::getFixedBody() const {
	return get().cast<FixedBody>();
}

void TNTFixedBody::updateRW(State &rwstate, const TNTIslandState &tntstate) const {
	// Nothing to update in RW
}

VelocityScrew6D<> TNTFixedBody::getVelocityW(const State &rwstate, const TNTIslandState &tntstate) const {
	return VelocityScrew6D<>();
}
