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
#include "RWPEIntegrator.hpp"

#include "RWPEConstraint.hpp"
#include "RWPEIntegratorEuler.hpp"
#include "RWPEIntegratorHeun.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEIntegrator::RWPEIntegrator():
	_body(NULL)
{
}

RWPEIntegrator::RWPEIntegrator(const RWPEBodyDynamic* body):
	_body(body)
{
}

RWPEIntegrator::~RWPEIntegrator()
{
}

const RWPEBodyDynamic* RWPEIntegrator::getBody() const {
	return _body;
}

RWPEBodyDynamic::RigidConfiguration* RWPEIntegrator::getConfiguration(const State &state) const {
	if (_body == NULL)
		RW_THROW("RWPEIntegrator (getConfiguration): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	return RWPEBodyDynamic::getDefaultConfiguration(_body->getRigidBody(),state);
}

void RWPEIntegrator::integrate(const std::list<const RWPEConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration, RWPEIslandState &state, const State &rwstate) const {
	if (_body == NULL)
		RW_THROW("RWPEIntegrator (integrate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench = getNetFT(constraints, gravity, configuration, state, rwstate);
	integrate(wrench,stepsize,configuration);
}

void RWPEIntegrator::positionUpdate(const std::list<const RWPEConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, RWPEBodyDynamic::RigidConfiguration &configuration, RWPEIslandState &state, const State &rwstate) const {
	if (_body == NULL)
		RW_THROW("RWPEIntegrator (positionUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench = getNetFT(constraints, gravity, configuration, state, rwstate);
	positionUpdate(wrench,stepsize,configuration);
}

void RWPEIntegrator::velocityUpdate(const std::list<const RWPEConstraint*> &constraints, const Vector3D<>& gravity, double stepsize, const RWPEBodyDynamic::RigidConfiguration &configuration0, RWPEBodyDynamic::RigidConfiguration &configurationH, const RWPEIslandState &state0, const RWPEIslandState &stateH, const State &rwstate, RWPELogUtil& log) const {
	if (_body == NULL)
		RW_THROW("RWPEIntegrator (velocityUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const Wrench6D<> wrench0 = getNetFT(constraints, gravity, configuration0, state0, rwstate);
	const Wrench6D<> wrenchH = getNetFT(constraints, gravity, configurationH, stateH, rwstate);
	if (log.doLog()) {
		std::map<std::string, Transform3D<> > posMap0;
		std::map<std::string, Transform3D<> > posMapH;
		std::map<std::string, Wrench6D<> > wrenchMap0;
		std::map<std::string, Wrench6D<> > wrenchMapH;
		posMap0[_body->get()->getName()] = configuration0.getWorldTcom();
		posMapH[_body->get()->getName()] = configurationH.getWorldTcom();
		wrenchMap0[_body->get()->getName()] = wrench0;
		wrenchMapH[_body->get()->getName()] = wrenchH;
		log.addPositions("Position at 0",posMap0,RWPE_LOCATION);
		log.addWrenches("Net Wrench at 0",wrenchMap0,RWPE_LOCATION);
		std::stringstream nstr;
		std::stringstream pstr;
		nstr << "Net Wrench at " << stepsize;
		pstr << "Position at " << stepsize;
		log.addPositions(pstr.str(),posMapH,RWPE_LOCATION);
		log.addWrenches(nstr.str(),wrenchMapH,RWPE_LOCATION);
	}
	velocityUpdate(wrench0,wrenchH,stepsize,configuration0,configurationH, log);
}

RWPEIntegrator::Factory::Factory():
	rw::common::ExtensionPoint<RWPEIntegrator>("rwsimlibs.rwpe.RWPEIntegrator", "RWPEIntegrator extension point.")
{
}

std::vector<std::string> RWPEIntegrator::Factory::getIntegrators() {
	std::vector<std::string> integrators;
	integrators.push_back("Euler");
	integrators.push_back("Heun");
	RWPEIntegrator::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		integrators.push_back( ext.getProperties().get("integratorID",ext.name) );
	}
	return integrators;
}

bool RWPEIntegrator::Factory::hasIntegrator(const std::string& integratorType) {
	if (integratorType == "Euler")
		return true;
	else if (integratorType == "Heun")
		return true;
	RWPEIntegrator::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("integratorID",ext.name) == integratorType)
            return true;
	}
	return false;
}

const RWPEIntegrator* RWPEIntegrator::Factory::makeIntegrator(const std::string& integratorType, const RWPEBodyDynamic* body) {
	if (integratorType == "Euler")
		return new RWPEIntegratorEuler(body);
	else if (integratorType == "Heun")
		return new RWPEIntegratorHeun(body);
	RWPEIntegrator::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("integratorID",ext->getName() ) == integratorType){
			const rw::common::Ptr<const RWPEIntegrator> base = ext->getObject().cast<const RWPEIntegrator>();
			return base->makeIntegrator(body);
		}
	}
	return NULL;
}

Wrench6D<> RWPEIntegrator::getNetFT(const std::list<const RWPEConstraint*> &constraints, const Vector3D<>& gravity, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEIslandState &state, const State &rwstate) const {
	const Vector3D<> R = configuration.getWorldTcom().P();
	Vector3D<> Fext = _body->getRigidBody()->getMass()*gravity + _body->getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = _body->getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
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

Wrench6D<> RWPEIntegrator::getExtFT(const std::list<const RWPEConstraint*> &constraints, const Vector3D<>& gravity, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEIslandState &state, const State &rwstate) const {
	const Vector3D<> R = configuration.getWorldTcom().P();
	Vector3D<> Fext = _body->getRigidBody()->getMass()*gravity + _body->getRigidBody()->getForceW(rwstate);
	Vector3D<> Next = _body->getRigidBody()->getTorqueW(rwstate);
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
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
