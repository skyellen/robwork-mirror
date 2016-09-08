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

#include <rwsim/contacts/Contact.hpp>
#include "LogContactSet.hpp"
#include "LogPositions.hpp"

using namespace rw::common;
using namespace rwsim::contacts;
using namespace rwsim::log;

LogContactSet::LogContactSet(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogContactSet::~LogContactSet() {
}

void LogContactSet::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("ContactSet");
	_contacts.resize(n);
	for (unsigned int i = 0; i < n; i++) {
		_contacts[i].read(iarchive,"");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogContactSet::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_contacts.size(),"ContactSet");
	BOOST_FOREACH(const Contact& c, _contacts) {
		c.write(oarchive,"");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogContactSet::getType() const {
	return getTypeID();
}

bool LogContactSet::operator==(const SimulatorLog &b) const {
	if (const LogContactSet* const entry = dynamic_cast<const LogContactSet*>(&b)) {
		if (_contacts != entry->_contacts)
			return false;
	}
	return SimulatorLogEntry::operator==(b);
}

std::list<SimulatorLogEntry::Ptr> LogContactSet::getLinkedEntries() const {
	// Link to last position entry in tree
	SimulatorLogScope* scope = getParent();
	while(scope != NULL) {
		std::vector<SimulatorLog::Ptr> children = scope->getChildren();
		std::vector<SimulatorLog::Ptr>::const_reverse_iterator it;
		for(it = children.rbegin(); it != children.rend(); it++) {
			const LogPositions::Ptr pos = (*it).cast<LogPositions>();
			if (pos != NULL) {
				return std::list<SimulatorLogEntry::Ptr>(1,pos);
			}
		}
		scope = scope->getParent();
	}
	return std::list<SimulatorLogEntry::Ptr>();
}

bool LogContactSet::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogContactSet::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogContactSet(parent));
};

std::string LogContactSet::getTypeID() {
	return "ContactSet";
}

const std::vector<Contact>& LogContactSet::getContacts() const {
	return _contacts;
}

const Contact& LogContactSet::getContact(std::size_t i) const {
	RW_ASSERT(i < _contacts.size());
	return _contacts[i];
}

void LogContactSet::setContacts(const std::vector<Contact>& contacts) {
	_contacts = contacts;
}

void LogContactSet::addContact(const Contact& contact) {
	_contacts.push_back(contact);
}

std::size_t LogContactSet::size() const {
	return _contacts.size();
}
