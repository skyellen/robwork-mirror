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
#include "LogPositions.hpp"
#include "LogVelocities.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogVelocities::LogVelocities(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogVelocities::~LogVelocities() {
}

void LogVelocities::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("Velocities");
	for (unsigned int i = 0; i < n; i++) {
		const std::string name = iarchive.readString("Name");
		iarchive.read(_velocities[name],"VelocityScrew6D");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogVelocities::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_velocities.size(),"Velocities");
	std::map<std::string,VelocityScrew6D<> >::const_iterator it;
	for (it = _velocities.begin(); it != _velocities.end(); it++) {
		oarchive.write(it->first,"Name");
		oarchive.write(it->second,"VelocityScrew6D");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogVelocities::getType() const {
	return getTypeID();
}

bool LogVelocities::operator==(const SimulatorLog &b) const {
	if (const LogVelocities* const entry = dynamic_cast<const LogVelocities*>(&b)) {
		if (*_positions != *entry->_positions)
			return false;
		if (_velocities != entry->_velocities)
			return false;
	}
	return SimulatorLogEntry::operator==(b);
}

std::list<SimulatorLogEntry::Ptr> LogVelocities::getLinkedEntries() const {
	if (_positions == NULL)
		return std::list<SimulatorLogEntry::Ptr>();
	else
		return std::list<SimulatorLogEntry::Ptr>(1,_positions);
}

bool LogVelocities::autoLink() {
	_positions = NULL;
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
			_positions = (*it).cast<LogPositions>();
			if (_positions != NULL) {
				found = true;
				break;
			}
		}
		find = scope;
		scope = scope->getParent();
	}
	return _positions != NULL;
}

SimulatorLogEntry::Ptr LogVelocities::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogVelocities(parent));
}

LogPositions::Ptr LogVelocities::getPositions() const {
	return _positions;
}

const std::map<std::string, VelocityScrew6D<> >& LogVelocities::getVelocities() const {
	return _velocities;
}

const VelocityScrew6D<>& LogVelocities::getVelocity(const std::string& name) const {
	const std::map<std::string,VelocityScrew6D<> >::const_iterator it = _velocities.find(name);
	RW_ASSERT(it != _velocities.end());
	return it->second;
}

bool LogVelocities::has(const std::string& name) const {
	return _velocities.find(name) != _velocities.end();
}

void LogVelocities::setVelocities(const std::map<std::string, VelocityScrew6D<> >& velocities) {
	_velocities = velocities;
}

void LogVelocities::setVelocity(const std::string& name, const VelocityScrew6D<>& velocity) {
	_velocities[name] = velocity;
}

std::size_t LogVelocities::size() const {
	return _velocities.size();
}

std::string LogVelocities::getTypeID() {
	return "Velocities";
}
