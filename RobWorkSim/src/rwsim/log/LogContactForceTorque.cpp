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

#include "SimulatorLogScope.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "LogContactForceTorque.hpp"
#include "LogContactSet.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogContactForceTorque::LogContactForceTorque(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogContactForceTorque::~LogContactForceTorque() {
}

void LogContactForceTorque::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("ContactForces");
	_forces.resize(n);
	for (unsigned int i = 0; i < n; i++) {
		iarchive.read(_forces[i].first,"Wrench6D");
		iarchive.read(_forces[i].second,"Wrench6D");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogContactForceTorque::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_forces.size(),"ContactForces");
	std::vector<std::pair<Wrench6D<>, Wrench6D<> > >::const_iterator it;
	for (it = _forces.begin(); it != _forces.end(); it++) {
		oarchive.write(it->first,"Wrench6D");
		oarchive.write(it->second,"Wrench6D");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogContactForceTorque::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogContactForceTorque::getLinkedEntries() const {
	if (_contacts == NULL)
		return std::list<SimulatorLogEntry::Ptr>();
	else
		return std::list<SimulatorLogEntry::Ptr>(1,_contacts);
}

bool LogContactForceTorque::autoLink() {
	_contacts = NULL;
	// Link to last position entry in tree
	SimulatorLogScope* scope = getParent();
	if (scope == NULL)
		return false;
	SimulatorLog* find = this;
	bool found = false;
	while(scope != NULL && !found) {
		// Find position of entry
		const std::vector<SimulatorLog::Ptr> children = scope->getChildren();
		std::vector<SimulatorLog::Ptr>::const_reverse_iterator it;
		for(it = children.rbegin(); it != children.rend(); it++) {
			if (it->isNull())
				continue;
			if (it->get() == find) {
				break;
			}
		}
		if (it != children.rend()) {
			if (it->get() == find)
				it++;
		}
		// Now search backwards
		for(; it != children.rend(); it++) {
			RW_ASSERT(*it != NULL);
			_contacts = (*it).cast<LogContactSet>();
			if (_contacts != NULL) {
				found = true;
				break;
			}
		}
		find = scope;
		scope = scope->getParent();
	}
	return _contacts != NULL;
}

SimulatorLogEntry::Ptr LogContactForceTorque::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogContactForceTorque(parent));
}

std::string LogContactForceTorque::getTypeID() {
	return "ContactForceTorque";
}

LogContactSet::Ptr LogContactForceTorque::getContacts() const {
	return _contacts;
}

Wrench6D<> LogContactForceTorque::getWrenchBodyA(std::size_t i) const {
	RW_ASSERT(i < _forces.size());
	return _forces[i].first;
}

Wrench6D<> LogContactForceTorque::getWrenchBodyB(std::size_t i) const {
	RW_ASSERT(i < _forces.size());
	return _forces[i].second;
}

void LogContactForceTorque::setWrench(std::size_t i, const Wrench6D<>& wrenchA, const Wrench6D<>& wrenchB) {
	if (i >= _forces.size()) {
		if (!_contacts.isNull())
			_forces.resize(std::max(i+1,_contacts->size()));
		else
			_forces.resize(i+1);
	}
	_forces[i].first = wrenchA;
	_forces[i].second = wrenchB;
}
