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

#include <rw/kinematics/State.hpp>

#include <rwsim/contacts/ContactStrategyTracking.hpp>

#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/SimulatorLogEntry.hpp>
#include <rwsim/log/LogCollisionResult.hpp>
#include <rwsim/log/LogContactForceTorque.hpp>
#include <rwsim/log/LogContactSet.hpp>
#include <rwsim/log/LogContactVelocities.hpp>
#include <rwsim/log/LogMessage.hpp>
#include <rwsim/log/LogPositions.hpp>
#include <rwsim/log/LogStep.hpp>
#include <rwsim/log/LogValues.hpp>
#include <rwsim/log/LogVelocities.hpp>
#include "RWPEIslandState.hpp"
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyKinematic.hpp"
#include "RWPEConstraint.hpp"
#include "RWPELogUtil.hpp"

#include "log/RWPELogContactTracking.hpp"
#include "RWPEContact.hpp"
#include "RWPEUtil.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::rwpe;

RWPELogUtil::RWPELogUtil():
	_scope(NULL)
{
}

RWPELogUtil::~RWPELogUtil() {
}

SimulatorLogScope* RWPELogUtil::makeScope(const std::string& name, const char* file, int line) {
	if (!doLog())
		return NULL;
	SimulatorLogScope::Ptr section = ownedPtr(new SimulatorLogScope(_scope));
	_scope->appendChild(section);
	section->setDescription(name);
	section->setFilename(file);
	section->setLineBegin(line);
	section->setLineEnd(line);
	return section.get();
}

void RWPELogUtil::setSimulatorLog(SimulatorLogScope::Ptr log) {
	_log = log;
	_scope = log.get();
}

bool RWPELogUtil::doLog() const {
	return _log != NULL;
}

void RWPELogUtil::beginStep(double time, const char* file, int line) {
	if (!doLog())
		return;
	LogStep::Ptr step = ownedPtr(new LogStep(_scope));
	_scope->appendChild(step);
	step->setTimeBegin(time);
	step->setFilename(file);
	step->setLineBegin(line);
	_scope = step.get();
}

void RWPELogUtil::endStep(double time, int line) {
	if (!doLog())
		return;
	LogStep* const step = dynamic_cast<LogStep*>(_scope);
	if (step == NULL)
		RW_THROW("Could not end step! - Not in correct scope.");
	step->setTimeEnd(time);
	step->setLineEnd(line);
	_scope = step->getParent();
}

void RWPELogUtil::beginSection(const std::string& name, const char* file, int line) {
	if (!doLog())
		return;
	SimulatorLogScope::Ptr section = ownedPtr(new SimulatorLogScope(_scope));
	_scope->appendChild(section);
	section->setDescription(name);
	section->setFilename(file);
	section->setLineBegin(line);
	_scope = section.get();
}

void RWPELogUtil::endSection(int line) {
	if (!doLog())
		return;
	SimulatorLogScope* const section = dynamic_cast<SimulatorLogScope*>(_scope);
	if (section == NULL)
		RW_THROW("Could not end section! - Not in correct scope.");
	section->setLineEnd(line);
	_scope = section->getParent();
}

void RWPELogUtil::addCollisionResults(const std::string& description, const std::vector<CollisionStrategy::Result>& results, const char* file, int line) {
	if (!doLog())
		return;
	LogCollisionResult::Ptr entry = ownedPtr(new LogCollisionResult(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->addResults(results);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addContacts(const std::string& description, const std::vector<rwsim::contacts::Contact>& contacts, const char* file, int line) {
	if (!doLog())
		return;
	LogContactSet::Ptr entry = ownedPtr(new LogContactSet(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setContacts(contacts);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addContacts(const std::string& description, const std::vector<const RWPEContact*>& contacts, const char* file, int line) {
	if (!doLog())
		return;
	LogContactSet::Ptr entry = ownedPtr(new LogContactSet(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		entry->addContact(contact->getContact());
	}
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addConstraints(const std::string& description, const std::vector<const RWPEConstraint*>& constraints, const char* file, int line) {
	if (!doLog())
		return;
	LogContactSet::Ptr entry = ownedPtr(new LogContactSet(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	//BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
		//entry->contacts.push_back(contact.getContact());
	//}
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addPositions(const std::string& description, const std::map<std::string,rw::math::Transform3D<> >& positions, const char* file, int line) {
	if (!doLog())
		return;
	LogPositions::Ptr entry = ownedPtr(new LogPositions(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setPositions(positions);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addPositions(const std::string& description, const RWPEBodyConstraintGraph* bc, const State& state, const char* file, int line) {
	if (!doLog())
		return;
	std::map<std::string, Transform3D<> > pos;
	/*BOOST_FOREACH(const RWPEBodyDynamic* const body, bc->getDynamicBodies()) {
		pos[body->get()->getName()] = body->getRigidBody()->wTbf(state);
	}
	BOOST_FOREACH(const RWPEBodyKinematic* const body, bc->getKinematicBodies()) {
		pos[body->get()->getName()] = body->getKinematicBody()->wTbf(state);
	}*/
	BOOST_FOREACH(const RWPEBody* const body, bc->getBodies()) {
		pos[body->get()->getName()] = body->get()->wTbf(state);
	}
	addPositions(description,pos,file,line);
}

void RWPELogUtil::addPositions(const std::string& description, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& state, const char* file, int line) {
	if (!doLog())
		return;
	std::map<std::string, Transform3D<> > pos;
	BOOST_FOREACH(const RWPEBody* const body, bc->getBodies()) {
		const BodyInfo& info = body->get()->getInfo();
		Transform3D<> wTb = body->getWorldTcom(state);
		wTb.P() -= wTb.R()*info.masscenter;
		pos[body->get()->getName()] = wTb;
	}
	addPositions(description,pos,file,line);
}

void RWPELogUtil::addVelocities(const std::string& description, const std::map<std::string,rw::math::VelocityScrew6D<> >& velocities, const char* file, int line) {
	if (!doLog())
		return;
	LogVelocities::Ptr entry = ownedPtr(new LogVelocities(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setVelocities(velocities);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addVelocities(const std::string& description, const RWPEBodyConstraintGraph* bc, const State& rwstate, const RWPEIslandState& islandState, const char* file, int line)  {
	if (!doLog())
		return;
	std::map<std::string, VelocityScrew6D<> > vel;
	BOOST_FOREACH(const RWPEBodyDynamic* const body, bc->getDynamicBodies()) {
		vel[body->get()->getName()] = body->getVelocityW(islandState);
	}
	BOOST_FOREACH(const RWPEBodyKinematic* const body, bc->getKinematicBodies()) {
		vel[body->get()->getName()] = body->getVelocityW(rwstate,islandState);
	}
	addVelocities(description,vel,file,line);
}

void RWPELogUtil::addWrenches(const std::string& description, const std::map<std::string, Wrench6D<> >& ft, const char* file, int line) {
	if (!doLog())
		return;
	// Just use the existing velocity type for now
	LogVelocities::Ptr entry = ownedPtr(new LogVelocities(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	std::map<std::string, Wrench6D<> >::const_iterator it;
	for (it = ft.begin(); it != ft.end(); it++) {
		entry->setVelocity(it->first,VelocityScrew6D<>(it->second.force(),EAA<>(it->second.torque())));
	}
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

void RWPELogUtil::addContactVelocities(const std::string& description, const std::vector<const RWPEContact*>& contacts, const State& rwstate, const RWPEIslandState& islandState, const char* file, int line)  {
	if (!doLog())
		return;
	LogContactVelocities::Ptr entry = ownedPtr(new LogContactVelocities(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setFilename(file);
	entry->setLine(line);
	RW_ASSERT(entry->autoLink());
	RW_ASSERT(entry->getContacts()->size() == contacts.size());
	for (std::size_t i = 0; i < contacts.size(); i++) {
		const RWPEContact* const contact = contacts[i];
		const Vector3D<> velP = contact->getVelocityParentW(islandState,rwstate).linear();
		const Vector3D<> velC = contact->getVelocityChildW(islandState,rwstate).linear();
		entry->setVelocity(i,velP,velC);
	}
}

void RWPELogUtil::addContactWrenches(const std::string& description, const std::vector<const RWPEContact*>& contacts, const RWPEIslandState& islandState, WrenchType type, const char* file, int line)  {
	if (!doLog())
		return;
	LogContactForceTorque::Ptr entry = ownedPtr(new LogContactForceTorque(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setFilename(file);
	entry->setLine(line);
	RW_ASSERT(entry->autoLink());
	RW_ASSERT(entry->getContacts()->size() == contacts.size());
	for (std::size_t i = 0; i < contacts.size(); i++) {
		const RWPEContact* const contact = contacts[i];
		Wrench6D<> wrench;
		switch (type) {
		case TOTAL:
			wrench = contact->getWrench(islandState);
			break;
		case APPLIED:
			wrench = contact->getWrenchApplied(islandState);
			break;
		case CONSTRAINT:
			wrench = contact->getWrenchConstraint(islandState);
			break;
		}
		entry->setWrench(i,wrench,wrench);
	}
}

void RWPELogUtil::addContactTracking(const std::string& description, const RWPEIslandState& islandState0, const RWPEIslandState& islandStateH, const char* file, int line)  {
	if (!doLog())
		return;
	RWPELogContactTracking::Ptr entry = ownedPtr(new RWPELogContactTracking(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setFilename(file);
	entry->setLine(line);
	RW_ASSERT(entry->autoLink());

	const std::vector<Contact> contacts0 = islandState0.getContacts();
	const std::vector<Contact> contactsH = islandStateH.getContacts();

	for (std::size_t i = 0; i < contacts0.size(); i++) {
		const ContactStrategyTracking::UserData::Ptr tracking0 = islandState0.getContactsTracking().getUserData(i);
		const RWPEUtil::RWPEUserData::Ptr rwpeData0 = tracking0.cast<const RWPEUtil::RWPEUserData>();
		RW_ASSERT(!rwpeData0.isNull());
		bool found = false;
		for (std::size_t j = 0; j < contactsH.size(); j++) {
			const ContactStrategyTracking::UserData::Ptr trackingH = islandStateH.getContactsTracking().getUserData(j);
			if (trackingH == RWPEUtil::MARK_NEW)
				continue;
			const RWPEUtil::RWPEUserData::Ptr rwpeDataH = trackingH.cast<const RWPEUtil::RWPEUserData>();
			RW_ASSERT(!rwpeDataH.isNull());
			if (rwpeData0 == rwpeDataH) {
				entry->before.push_back(contacts0[i]);
				entry->after.push_back(contactsH[j]);
				found = true;
				break;
			}
		}
		if (!found)
			entry->gone.push_back(contacts0[i]);
	}
	for (std::size_t i = 0; i < contactsH.size(); i++) {
		const ContactStrategyTracking::UserData::Ptr trackingH = islandStateH.getContactsTracking().getUserData(i);
		if (trackingH == RWPEUtil::MARK_NEW) {
			entry->added.push_back(contactsH[i]);
			continue;
		}
		const RWPEUtil::RWPEUserData::Ptr rwpeDataH = trackingH.cast<const RWPEUtil::RWPEUserData>();
		RW_ASSERT(!rwpeDataH.isNull());
		bool found = false;
		for (std::size_t j = 0; j < contacts0.size(); j++) {
			const ContactStrategyTracking::UserData::Ptr tracking0 = islandState0.getContactsTracking().getUserData(j);
			const RWPEUtil::RWPEUserData::Ptr rwpeData0 = tracking0.cast<const RWPEUtil::RWPEUserData>();
			RW_ASSERT(!rwpeData0.isNull());
			if (rwpeData0 == rwpeDataH) {
				found = true;
				break;
			}
		}
		if (!found)
			entry->added.push_back(contactsH[i]);
	}
	/*for (std::size_t j = 0; j < contactsH.size(); j++) {
		const ContactStrategyTracking::UserData::Ptr trackingH = islandStateH.getContactsTracking().getUserData(j);
		if (trackingH == RWPEUtil::MARK_NEW) {
			entry->added.push_back(contactsH[j]);
		}
	}*/
}

void RWPELogUtil::addValues(const std::string& description, const std::vector<double>& values, const std::vector<std::string>& labels, const char* file, int line) {
	if (!doLog())
		return;
	LogValues::Ptr entry = ownedPtr(new LogValues(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setData(labels,values);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
}

std::ostream& RWPELogUtil::log(const std::string& description, const char* file, int line) {
	if (!doLog()) {
		_dummyStream.str(std::string());
		return _dummyStream;
	}
	LogMessage::Ptr entry = ownedPtr(new LogMessage(_scope));
	_scope->appendChild(entry);
	entry->setDescription(description);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
	return entry->stream();
}

std::ostream& RWPELogUtil::log(const char* file, int line) {
	if (!doLog()) {
		_dummyStream.str(std::string());
		return _dummyStream;
	}
	LogMessage::Ptr entry = ownedPtr(new LogMessage(_scope));
	_scope->appendChild(entry);
	entry->setFilename(file);
	entry->setLine(line);
	entry->autoLink();
	return entry->stream();
}

RWPELogUtil* RWPELogUtil::parallel(const std::string& description, const char* file, int line) {
	if (!doLog())
		return NULL;
	const SimulatorLogScope::Ptr section = ownedPtr(new SimulatorLogScope(_scope));
	_scope->appendChild(section);
	section->setDescription("P "+description);
	section->setFilename(file);
	section->setLineBegin(line);
	RWPELogUtil* const asyncLog = new RWPELogUtil();
	asyncLog->setSimulatorLog(section);
	return asyncLog;
}
