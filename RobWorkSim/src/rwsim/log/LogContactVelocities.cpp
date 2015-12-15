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
#include "LogContactSet.hpp"
#include "LogContactVelocities.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogContactVelocities::LogContactVelocities(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogContactVelocities::~LogContactVelocities() {
}

void LogContactVelocities::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("ContactVelocities");
	_velocities.resize(n);
	for (unsigned int i = 0; i < n; i++) {
		iarchive.read(_velocities[i].first,"Vector3D");
		iarchive.read(_velocities[i].second,"Vector3D");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogContactVelocities::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_velocities.size(),"ContactVelocities");
	std::vector<std::pair<Vector3D<>, Vector3D<> > >::const_iterator it;
	for (it = _velocities.begin(); it != _velocities.end(); it++) {
		oarchive.write(it->first,"Vector3D");
		oarchive.write(it->second,"Vector3D");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogContactVelocities::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogContactVelocities::getLinkedEntries() const {
	if (_contacts == NULL)
		return std::list<SimulatorLogEntry::Ptr>();
	else
		return std::list<SimulatorLogEntry::Ptr>(1,_contacts);
}

bool LogContactVelocities::autoLink() {
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

SimulatorLogEntry::Ptr LogContactVelocities::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogContactVelocities(parent));
}

std::string LogContactVelocities::getTypeID() {
	return "ContactVelocities";
}

LogContactSet::Ptr LogContactVelocities::getContacts() const {
	return _contacts;
}

Vector3D<> LogContactVelocities::getVelocityBodyA(std::size_t i) const {
	RW_ASSERT(i < _velocities.size());
	return _velocities[i].first;
}

Vector3D<> LogContactVelocities::getVelocityBodyB(std::size_t i) const {
	RW_ASSERT(i < _velocities.size());
	return _velocities[i].second;
}

void LogContactVelocities::setVelocity(std::size_t i, const Vector3D<>& velocityA, const Vector3D<>& velocityB) {
	if (i >= _velocities.size()) {
		if (!_contacts.isNull())
			_velocities.resize(std::max(i+1,_contacts->size()));
		else
			_velocities.resize(i+1);
	}
	_velocities[i].first = velocityA;
	_velocities[i].second = velocityB;
}
