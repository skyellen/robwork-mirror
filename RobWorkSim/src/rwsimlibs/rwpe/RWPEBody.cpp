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

#include <rwsim/dynamics/Body.hpp>
#include "RWPEBody.hpp"
#include "RWPEIslandState.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEBody::RWPEBody(Body::Ptr body):
	_body(body)
{
}

RWPEBody::~RWPEBody() {
}

Body::Ptr RWPEBody::get() const {
	return _body;
}

RWPEBody::Configuration* RWPEBody::makeConfiguration(const State &state) const {
	return getDefaultConfiguration(_body,state);
}

Transform3D<> RWPEBody::getWorldTcom(const RWPEIslandState &state) const {
	const RWPEBody::Configuration* const config = state.getConfiguration(this);
	if (config != NULL)
		return config->getWorldTcom();
	else
		RW_THROW("RWPEBody (getWorldTcom): could not find a configuration for this body \"" << get()->getName() << "\".");
	return Transform3D<>::identity();
}

void RWPEBody::setWorldTcom(const Transform3D<> &wTcom, RWPEIslandState &state) const {
	RWPEBody::Configuration* const config = state.getConfiguration(this);
	if (config != NULL)
		config->setWorldTcom(wTcom);
	else
		RW_THROW("RWPEBody (setWorldTcom): could not find a configuration for this body \"" << get()->getName() << "\".");
}

void RWPEBody::reset(RWPEIslandState &state, const State &rwstate) const {
	RWPEBody::Configuration* const config = makeConfiguration(rwstate);
	state.setConfiguration(this,config);
}

RWPEBody::Configuration* RWPEBody::getDefaultConfiguration(rw::common::Ptr<const rwsim::dynamics::Body> body, const State &state) {
	Configuration* const config = new Configuration();
	Transform3D<> wTcom = body->wTbf(state);
	wTcom.P() += wTcom.R()*body->getInfo().masscenter;
	config->setWorldTcom(wTcom);
	return config;
}
