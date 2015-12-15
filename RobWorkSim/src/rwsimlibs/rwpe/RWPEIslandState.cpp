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

#include <boost/foreach.hpp>
#include "RWPEConstraint.hpp"
#include "RWPEFrictionModelData.hpp"
#include "RWPEIslandState.hpp"

#include "RWPEContact.hpp"
#include "RWPEUtil.hpp"

using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::rwpe;

RWPEIslandState::RWPEIslandState():
	_time(0),
	_timestep(0),
	_repeated(0)
{
};

RWPEIslandState::RWPEIslandState(const RWPEIslandState& state):
	_time(state.getTime()),
	_timestep(state.getLastTimeStep()),
	_repeated(state.getRepetitions()),
	_rwBodyToBody(state._rwBodyToBody),
	_constraintToAppliedWrench(state._constraintToAppliedWrench),
	_constraintToConstraintWrench(state._constraintToConstraintWrench),
	_contactData(state._contactData),
	_contacts(state._contacts),
	_tracking(state._tracking),
	_tempConstraints(state._tempConstraints),
	_bodyToTempConstraints(state._bodyToTempConstraints)
{
	std::map<const RWPEBody*, RWPEBody::Configuration*>::const_iterator it;
	for (it = state._bodyToConfiguration.begin(); it != state._bodyToConfiguration.end(); it++) {
		std::pair<const RWPEBody*, RWPEBody::Configuration*> entry = *it;
		_bodyToConfiguration[entry.first] = entry.second->clone();
	}
	std::map<const RWPEContact*, RWPEFrictionModelData*>::const_iterator itFriction;
	for (itFriction = state._contactToFrictionData.begin(); itFriction != state._contactToFrictionData.end(); itFriction++) {
		_contactToFrictionData[itFriction->first] = itFriction->second->clone();
	}
}

RWPEIslandState::~RWPEIslandState()
{
	std::map<const RWPEBody*, RWPEBody::Configuration*>::const_iterator it;
	for (it = _bodyToConfiguration.begin(); it != _bodyToConfiguration.end(); it++) {
		delete it->second;
	}
	std::map<const RWPEContact*, RWPEFrictionModelData*>::const_iterator itFriction;
	for (itFriction = _contactToFrictionData.begin(); itFriction != _contactToFrictionData.end(); itFriction++) {
		delete itFriction->second;
	}
};

RWPEIslandState& RWPEIslandState::operator=(const RWPEIslandState &state) {
	if (&state != this) {
		_time = state.getTime();
		_timestep = state.getLastTimeStep();
		_repeated = state.getRepetitions();
		_rwBodyToBody = state._rwBodyToBody;
		_constraintToAppliedWrench = state._constraintToAppliedWrench;
		_constraintToConstraintWrench = state._constraintToConstraintWrench;
		_contactData = state._contactData;
		_contacts = state._contacts;
		_tracking = state._tracking;
		_tempConstraints = state._tempConstraints;
		_bodyToTempConstraints = state._bodyToTempConstraints;
		std::map<const RWPEBody*, RWPEBody::Configuration*>::const_iterator it;
		for (it = _bodyToConfiguration.begin(); it != _bodyToConfiguration.end(); it++) {
			delete it->second;
		}
		_bodyToConfiguration.clear();
		for (it = state._bodyToConfiguration.begin(); it != state._bodyToConfiguration.end(); it++) {
			_bodyToConfiguration[it->first] = it->second->clone();
		}
		std::map<const RWPEContact*, RWPEFrictionModelData*>::const_iterator itFriction;
		for (itFriction = _contactToFrictionData.begin(); itFriction != _contactToFrictionData.end(); itFriction++) {
			delete itFriction->second;
		}
		_contactToFrictionData.clear();
		for (itFriction = state._contactToFrictionData.begin(); itFriction != state._contactToFrictionData.end(); itFriction++) {
			_contactToFrictionData[itFriction->first] = itFriction->second->clone();
		}
	}
	return *this;
}

double RWPEIslandState::getTime() const {
	return _time;
}

void RWPEIslandState::setTime(double time) {
	_time = time;
}

double RWPEIslandState::getLastTimeStep() const {
	return _timestep;
}

void RWPEIslandState::setLastTimeStep(double timestep) {
	_timestep = timestep;
}

std::size_t RWPEIslandState::getRepetitions() const {
	return _repeated;
}

void RWPEIslandState::setRepetitions(std::size_t repetitions) {
	_repeated = repetitions;
}

RWPEBody::Configuration* RWPEIslandState::getConfiguration(const RWPEBody* body) const {
	std::map<const RWPEBody*, RWPEBody::Configuration*>::const_iterator find = _bodyToConfiguration.find(body);
	if (find != _bodyToConfiguration.end())
		return find->second;
	return NULL;
}

void RWPEIslandState::setConfiguration(const RWPEBody* body, RWPEBody::Configuration* configuration) {
	if (_bodyToConfiguration.find(body) != _bodyToConfiguration.end())
		delete _bodyToConfiguration[body];
	_bodyToConfiguration[body] = configuration;
}

Wrench6D<> RWPEIslandState::getWrench(const RWPEConstraint* constraint) const {
	return getWrenchApplied(constraint)+getWrenchConstraint(constraint);
}

Wrench6D<> RWPEIslandState::getWrenchApplied(const RWPEConstraint* constraint) const {
	Wrench6D<> res;
	std::map<const RWPEConstraint*, Wrench6D<> >::const_iterator find = _constraintToAppliedWrench.find(constraint);
	if (find != _constraintToAppliedWrench.end())
		res = (*find).second;
	return res;
}

Wrench6D<> RWPEIslandState::getWrenchConstraint(const RWPEConstraint* constraint) const {
	Wrench6D<> res;
	std::map<const RWPEConstraint*, Wrench6D<> >::const_iterator find = _constraintToConstraintWrench.find(constraint);
	if (find != _constraintToConstraintWrench.end())
		res = (*find).second;
	return res;
}

void RWPEIslandState::setWrenchApplied(const RWPEConstraint* constraint, const Wrench6D<> wrench) {
	_constraintToAppliedWrench[constraint] = wrench;
}

void RWPEIslandState::setWrenchConstraint(const RWPEConstraint* constraint, const Wrench6D<> wrench) {
	_constraintToConstraintWrench[constraint] = wrench;
}

ContactDetectorData RWPEIslandState::getContactData() const {
	return _contactData;
}

void RWPEIslandState::setContactData(const ContactDetectorData& data) {
	_contactData = data;
}

std::vector<Contact> RWPEIslandState::getContacts() const {
	return _contacts;
}

ContactDetectorTracking RWPEIslandState::getContactsTracking() const {
	return _tracking;
}

bool RWPEIslandState::hasContacts() const {
	return _contacts.size() > 0;
}

void RWPEIslandState::setContacts(const std::vector<Contact>& contacts, const ContactDetectorTracking& data) {
	_contacts = contacts;
	_tracking = data;
}

RWPEFrictionModelData* RWPEIslandState::getFrictionData(const RWPEContact* contact) const {
	const std::map<const RWPEContact*, RWPEFrictionModelData*>::const_iterator it = _contactToFrictionData.find(contact);
	if (it != _contactToFrictionData.end()) {
		return it->second;
	}
	return NULL;
}

void RWPEIslandState::setFrictionData(const RWPEContact* contact, RWPEFrictionModelData* data) {
	clearFrictionData(contact);
	if (data != NULL) {
		_contactToFrictionData[contact] = data;
	}
}

void RWPEIslandState::clearFrictionData(const RWPEContact* contact) {
	RWPEFrictionModelData* const data = getFrictionData(contact);
	if (data != NULL)
		delete data;
	_contactToFrictionData.erase(contact);
}

std::list<RWPEConstraint*> RWPEIslandState::getTemporaryConstraints() const {
	return _tempConstraints;
}

std::list<const RWPEConstraint*> RWPEIslandState::getTemporaryConstraints(const RWPEBody* body) const {
	std::list<const RWPEConstraint*> list;
	{
		std::map<const RWPEBody*, std::list<const RWPEConstraint*> >::const_iterator it = _bodyToTempConstraints.find(body);
		if (it != _bodyToTempConstraints.end())
			list.insert(list.end(),(*it).second.begin(),(*it).second.end());
	}
	return list;
}

bool RWPEIslandState::hasTemporaryConstraints() const {
	return _tempConstraints.size() > 0;
}

void RWPEIslandState::addTemporaryConstraint(RWPEConstraint* constraint) {
	_tempConstraints.push_back(constraint);
	RW_ASSERT(constraint->getParent() != NULL);
	RW_ASSERT(constraint->getChild() != NULL);
	_bodyToTempConstraints[constraint->getParent()].push_back(constraint);
	_bodyToTempConstraints[constraint->getChild()].push_back(constraint);
}

void RWPEIslandState::removeTemporaryConstraint(const RWPEConstraint* constraint) {
	RWPEConstraint* found = NULL;
	BOOST_FOREACH(RWPEConstraint* tmp, _tempConstraints) {
		if (tmp == constraint) {
			found = tmp;
		}
	}
	if (found != NULL) {
		_tempConstraints.remove(found);
		_bodyToTempConstraints[constraint->getParent()].remove(constraint);
		_bodyToTempConstraints[constraint->getChild()].remove(constraint);
	}
	std::vector<Contact>::iterator it = _contacts.begin();
	for (std::size_t i = 0; i < _tracking.getInfo().size(); i++) {
		const ContactStrategyTracking::UserData::Ptr userData = _tracking.getUserData(i);
		const RWPEUtil::RWPEUserData* rwpeData = dynamic_cast<const RWPEUtil::RWPEUserData*>(userData.get());
		if (!rwpeData) {
			it++;
			continue;
		}
		if ((RWPEConstraint*)rwpeData->contact == constraint) {
			_tracking.remove(i);
			i--;
			it = _contacts.erase(it);
		} else {
			it++;
		}
	}
	const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
	if (contact == NULL)
		return;
	std::map<const RWPEContact*, RWPEFrictionModelData*>::iterator itFriction = _contactToFrictionData.find(contact);
	if (itFriction != _contactToFrictionData.end()) {
		delete itFriction->second;
		_contactToFrictionData.erase(itFriction);
	}
}

void RWPEIslandState::clearTemporaryConstraints() {
	while(_tempConstraints.size() > 0)
		removeTemporaryConstraint(_tempConstraints.front());
}
