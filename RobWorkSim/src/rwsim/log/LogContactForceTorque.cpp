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
	LogForceTorque(parent)
{
}

LogContactForceTorque::~LogContactForceTorque() {
}

std::string LogContactForceTorque::getType() const {
	return getTypeID();
}

bool LogContactForceTorque::operator==(const SimulatorLog &b) const {
	if (const LogContactForceTorque* const entry = dynamic_cast<const LogContactForceTorque*>(&b)) {
		if (*_contacts != *entry->_contacts)
			return false;
	}
	return LogForceTorque::operator==(b);
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

int LogContactForceTorque::sizeLinkedEntry() const {
	if (_contacts.isNull())
		return -1;
	return _contacts->size();
}

const std::string& LogContactForceTorque::getNameA(std::size_t i) const {
	if (_contacts.isNull())
		return _emptyStr;
	return _contacts->getContact(i).getNameA();
}

const std::string& LogContactForceTorque::getNameB(std::size_t i) const {
	if (_contacts.isNull())
		return _emptyStr;
	return _contacts->getContact(i).getNameB();
}

Vector3D<> LogContactForceTorque::getPositionA(std::size_t i) const {
	if (_contacts.isNull())
		return Vector3D<>::zero();
	return _contacts->getContact(i).getPointA();
}

Vector3D<> LogContactForceTorque::getPositionB(std::size_t i) const {
	if (_contacts.isNull())
		return Vector3D<>::zero();
	return _contacts->getContact(i).getPointB();
}

std::string LogContactForceTorque::getTypeID() {
	return "ContactForceTorque";
}

LogContactSet::Ptr LogContactForceTorque::getContacts() const {
	return _contacts;
}
