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

#include "TNTIslandState.hpp"
#include "TNTConstraint.hpp"

using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::tntphysics;

TNTIslandState::TNTIslandState():
	_time(0)
{
};

TNTIslandState::TNTIslandState(const TNTIslandState& state):
	_time(state.getTime()),
	_rwBodyToBody(state._rwBodyToBody),
	_constraintToAppliedWrench(state._constraintToAppliedWrench),
	_constraintToConstraintWrench(state._constraintToConstraintWrench),
	_contactData(state._contactData),
	_contacts(state._contacts),
	_tracking(state._tracking),
	_tempConstraints(state._tempConstraints),
	_bodyToTempConstraints(state._bodyToTempConstraints)
{
	std::map<const TNTBody*, TNTBody::Configuration*>::const_iterator it;
	for (it = state._bodyToConfiguration.begin(); it != state._bodyToConfiguration.end(); it++) {
		std::pair<const TNTBody*, TNTBody::Configuration*> entry = *it;
		_bodyToConfiguration[entry.first] = entry.second->clone();
	}
}

TNTIslandState::~TNTIslandState()
{
	std::map<const TNTBody*, TNTBody::Configuration*>::const_iterator it;
	for (it = _bodyToConfiguration.begin(); it != _bodyToConfiguration.end(); it++) {
		std::pair<const TNTBody*, TNTBody::Configuration*> entry = *it;
		delete entry.second;
	}
};

TNTIslandState& TNTIslandState::operator=(const TNTIslandState &state) {
	if (&state != this) {
		_time = state.getTime();
		_rwBodyToBody = state._rwBodyToBody;
		_constraintToAppliedWrench = state._constraintToAppliedWrench;
		_constraintToConstraintWrench = state._constraintToConstraintWrench;
		_contactData = state._contactData;
		_contacts = state._contacts;
		_tracking = state._tracking;
		_tempConstraints = state._tempConstraints;
		_bodyToTempConstraints = state._bodyToTempConstraints;
		std::map<const TNTBody*, TNTBody::Configuration*>::const_iterator it;
		for (it = state._bodyToConfiguration.begin(); it != state._bodyToConfiguration.end(); it++) {
			std::pair<const TNTBody*, TNTBody::Configuration*> entry = *it;
			_bodyToConfiguration[entry.first] = entry.second->clone();
		}
	}
	return *this;
}

double TNTIslandState::getTime() const {
	return _time;
}

void TNTIslandState::setTime(double time) {
	_time = time;
}

TNTBody::Configuration* TNTIslandState::getConfiguration(const TNTBody* body) const {
	std::map<const TNTBody*, TNTBody::Configuration*>::const_iterator find = _bodyToConfiguration.find(body);
	if (find != _bodyToConfiguration.end())
		return find->second;
	return NULL;
}

void TNTIslandState::setConfiguration(const TNTBody* body, TNTBody::Configuration* configuration) {
	if (_bodyToConfiguration.find(body) != _bodyToConfiguration.end())
		delete _bodyToConfiguration[body];
	_bodyToConfiguration[body] = configuration;
}

Wrench6D<> TNTIslandState::getWrench(const TNTConstraint* constraint) const {
	return getWrenchApplied(constraint)+getWrenchConstraint(constraint);
}

Wrench6D<> TNTIslandState::getWrenchApplied(const TNTConstraint* constraint) const {
	Wrench6D<> res;
	std::map<const TNTConstraint*, Wrench6D<> >::const_iterator find = _constraintToAppliedWrench.find(constraint);
	if (find != _constraintToAppliedWrench.end())
		res = (*find).second;
	return res;
}

Wrench6D<> TNTIslandState::getWrenchConstraint(const TNTConstraint* constraint) const {
	Wrench6D<> res;
	std::map<const TNTConstraint*, Wrench6D<> >::const_iterator find = _constraintToConstraintWrench.find(constraint);
	if (find != _constraintToConstraintWrench.end())
		res = (*find).second;
	return res;
}

void TNTIslandState::setWrenchApplied(const TNTConstraint* constraint, const Wrench6D<> wrench) {
	_constraintToAppliedWrench[constraint] = wrench;
}

void TNTIslandState::setWrenchConstraint(const TNTConstraint* constraint, const Wrench6D<> wrench) {
	_constraintToConstraintWrench[constraint] = wrench;
}

ContactDetectorData TNTIslandState::getContactData() const {
	return _contactData;
}

void TNTIslandState::setContactData(const ContactDetectorData& data) {
	_contactData = data;
}

std::vector<Contact> TNTIslandState::getContacts() const {
	return _contacts;
}

ContactDetectorTracking TNTIslandState::getContactsTracking() const {
	return _tracking;
}

void TNTIslandState::setContacts(const std::vector<Contact>& contacts, const ContactDetectorTracking& data) {
	_contacts = contacts;
	_tracking = data;
}

std::list<TNTConstraint*> TNTIslandState::getTemporaryConstraints() const {
	return _tempConstraints;
}

std::list<const TNTConstraint*> TNTIslandState::getTemporaryConstraints(const TNTBody* body) const {
	std::list<const TNTConstraint*> list;
	{
		std::map<const TNTBody*, std::list<const TNTConstraint*> >::const_iterator it = _bodyToTempConstraints.find(body);
		if (it != _bodyToTempConstraints.end())
			list.insert(list.end(),(*it).second.begin(),(*it).second.end());
	}
	return list;
}

void TNTIslandState::addTemporaryConstraint(TNTConstraint* constraint) {
	_tempConstraints.push_back(constraint);
	RW_ASSERT(constraint->getParent() != NULL);
	RW_ASSERT(constraint->getChild() != NULL);
	_bodyToTempConstraints[constraint->getParent()].push_back(constraint);
	_bodyToTempConstraints[constraint->getChild()].push_back(constraint);
}

void TNTIslandState::removeTemporaryConstraint(const TNTConstraint* constraint) {
	TNTConstraint* found = NULL;
	BOOST_FOREACH(TNTConstraint* tmp, _tempConstraints) {
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
		const void* const userData = _tracking.getUserData(i);
		if (userData == constraint) {
			_tracking.remove(i);
			i--;
			it = _contacts.erase(it);
		} else {
			it++;
		}
	}
}

void TNTIslandState::clearTemporaryConstraints() {
	while(_tempConstraints.size() > 0)
		removeTemporaryConstraint(_tempConstraints.front());
}
