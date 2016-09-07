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

#include "LogConstraintForceTorque.hpp"
#include "LogConstraints.hpp"
#include "SimulatorLogScope.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogConstraintForceTorque::LogConstraintForceTorque(SimulatorLogScope* parent):
	LogForceTorque(parent)
{
}

LogConstraintForceTorque::~LogConstraintForceTorque() {
}

std::string LogConstraintForceTorque::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogConstraintForceTorque::getLinkedEntries() const {
	if (_constraints == NULL)
		return std::list<SimulatorLogEntry::Ptr>();
	else
		return std::list<SimulatorLogEntry::Ptr>(1,_constraints);
}

bool LogConstraintForceTorque::autoLink() {
	_constraints = NULL;
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
			_constraints = (*it).cast<LogConstraints>();
			if (_constraints != NULL) {
				found = true;
				break;
			}
		}
		find = scope;
		scope = scope->getParent();
	}
	return _constraints != NULL;
}

SimulatorLogEntry::Ptr LogConstraintForceTorque::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogConstraintForceTorque(parent));
}

const std::string& LogConstraintForceTorque::getNameA(std::size_t i) const {
	if (_constraints.isNull())
		return _emptyStr;
	return _constraints->getConstraint(i).frameA;
}

const std::string& LogConstraintForceTorque::getNameB(std::size_t i) const {
	if (_constraints.isNull())
		return _emptyStr;
	return _constraints->getConstraint(i).frameB;
}

Vector3D<> LogConstraintForceTorque::getPositionA(std::size_t i) const {
	if (_constraints.isNull())
		return Vector3D<>::zero();
	return _constraints->getConstraint(i).posA;
}

Vector3D<> LogConstraintForceTorque::getPositionB(std::size_t i) const {
	if (_constraints.isNull())
		return Vector3D<>::zero();
	return _constraints->getConstraint(i).posB;
}

int LogConstraintForceTorque::sizeLinkedEntry() const {
	if (_constraints.isNull())
		return -1;
	else
		return _constraints->size();
}

std::string LogConstraintForceTorque::getTypeID() {
	return "ConstraintForceTorque";
}

LogConstraints::Ptr LogConstraintForceTorque::getConstraints() const {
	return _constraints;
}
