/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "LogConstraints.hpp"
#include "LogPositions.hpp"
#include "SimulatorLogScope.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogConstraints::LogConstraints(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogConstraints::~LogConstraints() {
}

void LogConstraints::read(InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("Constraints");
	_constraints.resize(n);
	for (unsigned int i = 0; i < n; i++) {
		Constraint& c = _constraints[i];
		c.frameA = iarchive.readString("NameA");
		c.frameB = iarchive.readString("NameB");
		c.type = iarchive.readString("Type");
		iarchive.read(c.posA,"PosA");
		iarchive.read(c.posB,"PosB");
		iarchive.read(c.rotAlin,"RotAlin");
		iarchive.read(c.rotBlin,"RotBlin");
		iarchive.read(c.rotAang,"RotAang");
		iarchive.read(c.rotBang,"RotBang");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogConstraints::write(OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_constraints.size(),"Constraints");
	BOOST_FOREACH(const Constraint& c, _constraints) {
		oarchive.write(c.frameA,"NameA");
		oarchive.write(c.frameB,"NameB");
		oarchive.write(c.type,"Type");
		oarchive.write(c.posA,"PosA");
		oarchive.write(c.posB,"posB");
		oarchive.write(c.rotAlin,"RotAlin");
		oarchive.write(c.rotBlin,"RotBlin");
		oarchive.write(c.rotAang,"RotAang");
		oarchive.write(c.rotBang,"RotBang");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogConstraints::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogConstraints::getLinkedEntries() const {
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

bool LogConstraints::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogConstraints::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogConstraints(parent));
}

std::string LogConstraints::getTypeID() {
	return "LogConstraints";
}

std::size_t LogConstraints::size() const {
	return _constraints.size();
}

void LogConstraints::addConstraint(const Constraint& constraint) {
	_constraints.push_back(constraint);
}

const std::vector<LogConstraints::Constraint>& LogConstraints::getConstraints() const {
	return _constraints;
}

const LogConstraints::Constraint& LogConstraints::getConstraint(std::size_t i) const {
	RW_ASSERT(i < _constraints.size());
	return _constraints[i];
}
