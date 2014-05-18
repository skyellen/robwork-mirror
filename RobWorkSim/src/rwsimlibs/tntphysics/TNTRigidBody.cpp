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

#include "TNTRigidBody.hpp"
#include "TNTIntegrator.hpp"
#include "TNTIslandState.hpp"

#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTRigidBody::TNTRigidBody(rw::common::Ptr<RigidBody> body):
	TNTBody(body),
	_rbody(body),
	_integrator(TNTIntegrator::Factory::makeIntegrator(body->getInfo().integratorType,this))
{
	if (_integrator == NULL)
		RW_THROW("TNTRigidBody: could not create instance for body \"" << body->getName() << "\" - integrator not found.");
}

TNTRigidBody::~TNTRigidBody() {
	delete _integrator;
}

rw::common::Ptr<const RigidBody> TNTRigidBody::getRigidBody() const {
	return get().cast<const RigidBody>();
}

TNTBody::Configuration* TNTRigidBody::makeConfiguration(const State &rwstate) const {
	return _integrator->getConfiguration(rwstate);
}

const TNTRigidBody::RigidConfiguration* TNTRigidBody::getConfiguration(const TNTIslandState &tntstate) const {
	return dynamic_cast<const TNTRigidBody::RigidConfiguration*>(tntstate.getConfiguration(this));
}

void TNTRigidBody::reset(TNTIslandState &tntstate, const State &rwstate) const {
	tntstate.setConfiguration(this,makeConfiguration(rwstate));
}

void TNTRigidBody::updateRW(State &rwstate, const TNTIslandState &tntstate) const {
	const RigidBody::Ptr rwbody = _rbody;
	const RigidConfiguration* const config = dynamic_cast<RigidConfiguration*>(tntstate.getConfiguration(this));
	if (config == NULL)
		RW_THROW("TNTRigidBody (updateRW): expected RigidConfiguration in state for body \"" << rwbody->getName() << "\".");
	Transform3D<> wTb = config->getWorldTcom();
	const VelocityScrew6D<> velW = config->getVelocity();
	wTb.P() -= wTb.R()*rwbody->getInfo().masscenter;

	MovableFrame* const frame = dynamic_cast<MovableFrame*>(rwbody->getBodyFrame());
	if (frame == NULL)
		RW_THROW("TNTRigidBody (updateRW): can not update motion for body \"" << rwbody->getName() << "\" as it is not a movable frame.");

	const Transform3D<> wTp = rwbody->getWTParent(rwstate);
	frame->setTransform(inverse(wTp)*wTb,rwstate);
	rwbody->setLinVelW(velW.linear(),rwstate);
	rwbody->setAngVelW(velW.angular().axis()*velW.angular().angle(),rwstate);
}

VelocityScrew6D<> TNTRigidBody::getVelocityW(const TNTIslandState &tntstate) const {
	const RigidConfiguration* const config = getConfiguration(tntstate);
	if (config == NULL)
		RW_THROW("TNTRigidBody (getVelocityW): configuration must be of type RigidConfiguration!");
	return config->getVelocity();
}

VelocityScrew6D<> TNTRigidBody::getVelocityW(const State &rwstate, const TNTIslandState &tntstate) const {
	return getVelocityW(tntstate);
}

void TNTRigidBody::setVelocityW(const rw::math::VelocityScrew6D<>& velW, TNTIslandState &tntstate) const {
	RigidConfiguration* const config = dynamic_cast<RigidConfiguration*>(tntstate.getConfiguration(this));
	config->setVelocity(velW);
}

TNTRigidBody::RigidConfiguration* TNTRigidBody::getDefaultConfiguration(rw::common::Ptr<const rwsim::dynamics::RigidBody> body, const State &state) {
	RigidConfiguration* const config = new RigidConfiguration();
	Transform3D<> wTcom = body->wTbf(state);
	wTcom.P() += wTcom.R()*body->getInfo().masscenter;
	config->setWorldTcom(wTcom);
	const Vector3D<> linVel = body->getLinVelW(state);
	const EAA<> angVel = EAA<>(body->getAngVelW(state));
	config->setVelocity(VelocityScrew6D<>(linVel,angVel));
	return config;
}

const TNTIntegrator* TNTRigidBody::getIntegrator() const {
	return _integrator;
}
