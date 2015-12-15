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

#include <rwsim/dynamics/RigidBody.hpp>
#include "RWPEBodyDynamic.hpp"

#include "RWPEConstraint.hpp"
#include "RWPEIntegrator.hpp"
#include "RWPEIslandState.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEBodyDynamic::RWPEBodyDynamic(rw::common::Ptr<RigidBody> body):
	RWPEBody(body),
	_rbody(body),
	_integrator(RWPEIntegrator::Factory::makeIntegrator(body->getInfo().integratorType,this))
{
	if (_integrator == NULL)
		RW_THROW("RWPEBodyDynamic: could not create instance for body \"" << body->getName() << "\" - integrator not found.");
}

RWPEBodyDynamic::~RWPEBodyDynamic() {
	delete _integrator;
}

rw::common::Ptr<const RigidBody> RWPEBodyDynamic::getRigidBody() const {
	return get().cast<const RigidBody>();
}

RWPEBody::Configuration* RWPEBodyDynamic::makeConfiguration(const State &rwstate) const {
	return _integrator->getConfiguration(rwstate);
}

const RWPEBodyDynamic::RigidConfiguration* RWPEBodyDynamic::getConfiguration(const RWPEIslandState &state) const {
	return dynamic_cast<const RWPEBodyDynamic::RigidConfiguration*>(state.getConfiguration(this));
}

Wrench6D<> RWPEBodyDynamic::getNetWrench(const Vector3D<>& gravity, const std::list<RWPEConstraint*>& constraints, const RWPEIslandState &state, const State& rwstate) const {
	const RWPEBodyDynamic::RigidConfiguration* const config = getConfiguration(state);
	const Vector3D<> R = config->getWorldTcom().P();
	Vector3D<> Fext = gravity*getRigidBody()->getMass()+getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(RWPEConstraint* constraint, constraints) {
		const Wrench6D<> wrench = constraint->getWrench(state);
		if (constraint->getParent() == this) {
			const Vector3D<> pos = constraint->getPositionParentW(state);
			Fext += wrench.force();
			Next += wrench.torque()+cross(pos-R,wrench.force());
		} else if (constraint->getChild() == this) {
			const Vector3D<> pos = constraint->getPositionChildW(state);
			Fext += -wrench.force();
			Next += -wrench.torque()+cross(pos-R,-wrench.force());
		}
	}
	return Wrench6D<>(Fext,Next);
}

Wrench6D<> RWPEBodyDynamic::getExternalWrench(const Vector3D<>& gravity, const std::list<RWPEConstraint*>& constraints, const RWPEIslandState &state, const State& rwstate) const {
	const RWPEBodyDynamic::RigidConfiguration* const config = getConfiguration(state);
	const Vector3D<> R = config->getWorldTcom().P();
	Vector3D<> Fext = gravity*getRigidBody()->getMass()+getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(RWPEConstraint* constraint, constraints) {
		const Wrench6D<> wrench = constraint->getWrenchApplied(state);
		if (constraint->getParent() == this) {
			const Vector3D<> pos = constraint->getPositionParentW(state);
			Fext += wrench.force();
			Next += wrench.torque()+cross(pos-R,wrench.force());
		} else if (constraint->getChild() == this) {
			const Vector3D<> pos = constraint->getPositionChildW(state);
			Fext += -wrench.force();
			Next += -wrench.torque()+cross(pos-R,-wrench.force());
		}
	}
	return Wrench6D<>(Fext,Next);
}

void RWPEBodyDynamic::reset(RWPEIslandState &state, const State &rwstate) const {
	state.setConfiguration(this,makeConfiguration(rwstate));
}

void RWPEBodyDynamic::updateRW(State &rwstate, const RWPEIslandState &state) const {
	const RigidBody::Ptr rwbody = _rbody;
	const RigidConfiguration* const config = dynamic_cast<RigidConfiguration*>(state.getConfiguration(this));
	if (config == NULL)
		RW_THROW("RWPEBodyDynamic (updateRW): expected RigidConfiguration in state for body \"" << rwbody->getName() << "\".");
	Transform3D<> wTb = config->getWorldTcom();
	const VelocityScrew6D<> velW = config->getVelocity();
	wTb.P() -= wTb.R()*rwbody->getInfo().masscenter;

	MovableFrame* const frame = dynamic_cast<MovableFrame*>(rwbody->getBodyFrame());
	if (frame == NULL)
		RW_THROW("RWPEBodyDynamic (updateRW): can not update motion for body \"" << rwbody->getName() << "\" as it is not a movable frame.");

	const Transform3D<> wTp = rwbody->getWTParent(rwstate);
	frame->setTransform(inverse(wTp)*wTb,rwstate);
	rwbody->setLinVelW(velW.linear(),rwstate);
	rwbody->setAngVelW(velW.angular().axis()*velW.angular().angle(),rwstate);
}

VelocityScrew6D<> RWPEBodyDynamic::getVelocityW(const RWPEIslandState &state) const {
	const RigidConfiguration* const config = getConfiguration(state);
	if (config == NULL)
		RW_THROW("RWPEBodyDynamic (getVelocityW): configuration must be of type RigidConfiguration!");
	return config->getVelocity();
}

VelocityScrew6D<> RWPEBodyDynamic::getVelocityW(const State &rwstate, const RWPEIslandState &state) const {
	return getVelocityW(state);
}

void RWPEBodyDynamic::setVelocityW(const rw::math::VelocityScrew6D<>& velW, RWPEIslandState &state) const {
	RigidConfiguration* const config = dynamic_cast<RigidConfiguration*>(state.getConfiguration(this));
	config->setVelocity(velW);
}

double RWPEBodyDynamic::getKineticEnergy(const RigidConfiguration& configuration) const {
	const double mass = _rbody->getMass();
	const Transform3D<>& wTb = configuration.getWorldTcom();
	const VelocityScrew6D<>& vel = configuration.getVelocity();
	const Vector3D<> angVel(vel[3],vel[4],vel[5]);
	const InertiaMatrix<>& inertia = wTb.R()*_rbody->getBodyInertia()*inverse(wTb.R());
	return (mass*dot(vel.linear(),vel.linear())+dot(angVel,inertia*angVel))*0.5;
}

double RWPEBodyDynamic::getKineticEnergy(const RWPEIslandState& state) const {
	return getKineticEnergy(*getConfiguration(state));
}

RWPEBodyDynamic::RigidConfiguration* RWPEBodyDynamic::getDefaultConfiguration(rw::common::Ptr<const rwsim::dynamics::RigidBody> body, const State &state) {
	RigidConfiguration* const config = new RigidConfiguration();
	Transform3D<> wTcom = body->wTbf(state);
	wTcom.P() += wTcom.R()*body->getInfo().masscenter;
	config->setWorldTcom(wTcom);
	const Vector3D<> linVel = body->getLinVelW(state);
	const EAA<> angVel = EAA<>(body->getAngVelW(state));
	config->setVelocity(VelocityScrew6D<>(linVel,angVel));
	return config;
}

const RWPEIntegrator* RWPEBodyDynamic::getIntegrator() const {
	return _integrator;
}
