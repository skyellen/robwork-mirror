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

#include "TNTIntegrator.hpp"
#include "TNTIntegratorEuler.hpp"
#include "TNTIntegratorHeun.hpp"

#include "TNTConstraint.hpp"
#include "TNTIslandState.hpp"

#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTIntegrator::TNTIntegrator():
	_body(NULL)
{
}

TNTIntegrator::TNTIntegrator(const TNTRigidBody* body):
	_body(body)
{
}

TNTIntegrator::~TNTIntegrator()
{
}

const TNTRigidBody* TNTIntegrator::getBody() const {
	return _body;
}

TNTRigidBody::RigidConfiguration* TNTIntegrator::getConfiguration(const State &state) const {
	if (_body == NULL)
		RW_THROW("TNTIntegrator (getConfiguration): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	return TNTRigidBody::getDefaultConfiguration(_body->getRigidBody(),state);
}

void TNTIntegrator::integrate(const std::list<const TNTConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, TNTRigidBody::RigidConfiguration &configuration, TNTIslandState &state, const State &rwstate) const {
	if (_body == NULL)
		RW_THROW("TNTIntegrator (integrate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench = getNetFT(constraints, gravity, configuration, state, rwstate);
	integrate(wrench,stepsize,configuration);
}

void TNTIntegrator::positionUpdate(const std::list<const TNTConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, TNTRigidBody::RigidConfiguration &configuration, TNTIslandState &state, const State &rwstate) const {
	if (_body == NULL)
		RW_THROW("TNTIntegrator (positionUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench = getNetFT(constraints, gravity, configuration, state, rwstate);
	positionUpdate(wrench,stepsize,configuration);
}

void TNTIntegrator::velocityUpdate(const std::list<const TNTConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, const TNTRigidBody::RigidConfiguration &configuration0, TNTRigidBody::RigidConfiguration &configurationH, const TNTIslandState &state0, const TNTIslandState &stateH, const State &rwstate) const {
	if (_body == NULL)
		RW_THROW("TNTIntegrator (velocityUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench0 = getNetFT(constraints, gravity, configuration0, state0, rwstate);
	const Wrench6D<> wrenchH = getNetFT(constraints, gravity, configurationH, stateH, rwstate);
	velocityUpdate(wrench0,wrenchH,stepsize,configuration0,configurationH);
}

TNTIntegrator::Factory::Factory():
	rw::common::ExtensionPoint<TNTIntegrator>("rwsimlibs.tntphysics.TNTIntegrator", "TNTIntegrator extension point.")
{
}

std::vector<std::string> TNTIntegrator::Factory::getIntegrators() {
	std::vector<std::string> integrators;
	integrators.push_back("Euler");
	integrators.push_back("Heun");
	TNTIntegrator::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		integrators.push_back( ext.getProperties().get("integratorID",ext.name) );
	}
	return integrators;
}

bool TNTIntegrator::Factory::hasIntegrator(const std::string& integratorType) {
	if (integratorType == "Euler")
		return true;
	else if (integratorType == "Heun")
		return true;
	TNTIntegrator::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("integratorID",ext.name) == integratorType)
            return true;
	}
	return false;
}

const TNTIntegrator* TNTIntegrator::Factory::makeIntegrator(const std::string& integratorType, const TNTRigidBody* body) {
	if (integratorType == "Euler")
		return new TNTIntegratorEuler(body);
	else if (integratorType == "Heun")
		return new TNTIntegratorHeun(body);
	TNTIntegrator::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("integratorID",ext->getName() ) == integratorType){
			const rw::common::Ptr<const TNTIntegrator> base = ext->getObject().cast<const TNTIntegrator>();
			return base->makeIntegrator(body);
		}
	}
	return NULL;
}

Wrench6D<> TNTIntegrator::getNetFT(const std::list<const TNTConstraint*> &constraints, const Vector3D<>& gravity, const TNTRigidBody::RigidConfiguration &configuration, const TNTIslandState &state, const State &rwstate) const {
	const Vector3D<> R = configuration.getWorldTcom().P();
	Vector3D<> Fext = _body->getRigidBody()->getMass()*gravity + _body->getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = _body->getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const Wrench6D<> wrench = state.getWrench(constraint);
		if (constraint->getParent() == _body) {
			const Vector3D<> pos = constraint->getPositionParentW(state);
			Fext += wrench.force();
			Next += wrench.torque()+cross(pos-R,wrench.force());
		} else if (constraint->getChild() == _body) {
			const Vector3D<> pos = constraint->getPositionChildW(state);
			Fext += -wrench.force();
			Next += -wrench.torque()+cross(pos-R,-wrench.force());
		}
	}
	return Wrench6D<>(Fext,Next);
}

Wrench6D<> TNTIntegrator::getExtFT(const std::list<const TNTConstraint*> &constraints, const Vector3D<>& gravity, const TNTRigidBody::RigidConfiguration &configuration, const TNTIslandState &state, const State &rwstate) const {
	const Vector3D<> R = configuration.getWorldTcom().P();
	Vector3D<> Fext = _body->getRigidBody()->getMass()*gravity + _body->getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = _body->getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const Wrench6D<> wrench = state.getWrenchApplied(constraint);
		if (constraint->getParent() == _body) {
			const Vector3D<> pos = constraint->getPositionParentW(state);
			Fext += wrench.force();
			Next += wrench.torque()+cross(pos-R,wrench.force());
		} else if (constraint->getChild() == _body) {
			const Vector3D<> pos = constraint->getPositionChildW(state);
			Fext += -wrench.force();
			Next += -wrench.torque()+cross(pos-R,-wrench.force());
		}
	}
	return Wrench6D<>(Fext,Next);
}
