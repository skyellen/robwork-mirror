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

#include "SimulatorLogEntry.hpp"
#include "SimulatorLogScope.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "LogCollisionResult.hpp"
#include "LogConstraintForceTorque.hpp"
#include "LogConstraints.hpp"
#include "LogContactForceTorque.hpp"
#include "LogContactSet.hpp"
#include "LogContactVelocities.hpp"
#include "LogEquationSystem.hpp"
#include "LogMessage.hpp"
#include "LogPositions.hpp"
#include "LogVelocities.hpp"
#include "LogValues.hpp"

using namespace rw::common;
using namespace rwsim::log;

SimulatorLogEntry::SimulatorLogEntry(SimulatorLogScope* parent):
	SimulatorLog(parent),
	_line(-1)
{
}

SimulatorLogEntry::~SimulatorLogEntry() {
}

void SimulatorLogEntry::read(class InputArchive& iarchive, const std::string& id) {
	_line = iarchive.readInt("Line");
	SimulatorLog::read(iarchive,id);
}

void SimulatorLogEntry::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_line,"Line");
	SimulatorLog::write(oarchive,id);
}

std::size_t SimulatorLogEntry::children() const {
	return 0;
}

bool SimulatorLogEntry::operator==(const SimulatorLog &b) const {
	if (const SimulatorLogEntry* const entry = dynamic_cast<const SimulatorLogEntry*>(&b)) {
		if (_line != entry->_line)
			return false;
	}
	return SimulatorLog::operator==(b);
}

int SimulatorLogEntry::line() const {
	return _line;
}

void SimulatorLogEntry::setLine(int line) {
	_line = line;
}

SimulatorLogEntry::Factory::Factory():
	ExtensionPoint<SimulatorLogEntry>("rwsim.log.SimulatorLogEntry", "SimulatorLogEntry extension point.")
{
}

std::vector<std::string> SimulatorLogEntry::Factory::getEntryTypes() {
	std::vector<std::string> res;
	res.push_back(LogCollisionResult::getTypeID());
	res.push_back(LogConstraintForceTorque::getTypeID());
	res.push_back(LogConstraints::getTypeID());
	res.push_back(LogContactForceTorque::getTypeID());
	res.push_back(LogContactSet::getTypeID());
	res.push_back(LogContactVelocities::getTypeID());
	res.push_back(LogEquationSystem::getTypeID());
	res.push_back(LogPositions::getTypeID());
	res.push_back(LogVelocities::getTypeID());
	res.push_back(LogMessage::getTypeID());
	res.push_back(LogValues::getTypeID());
	SimulatorLogEntry::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		res.push_back( ext.getProperties().get("entryType",ext.name) );
	}
	return res;
}

bool SimulatorLogEntry::Factory::hasEntryType(const std::string& entryType) {
	if (entryType == LogCollisionResult::getTypeID())
		return true;
	else if (entryType == LogConstraintForceTorque::getTypeID())
		return true;
	else if (entryType == LogConstraints::getTypeID())
		return true;
	else if (entryType == LogContactForceTorque::getTypeID())
		return true;
	else if (entryType == LogContactSet::getTypeID())
		return true;
	else if (entryType == LogContactVelocities::getTypeID())
		return true;
	else if (entryType == LogEquationSystem::getTypeID())
		return true;
	else if (entryType == LogPositions::getTypeID())
		return true;
	else if (entryType == LogVelocities::getTypeID())
		return true;
	else if (entryType == LogMessage::getTypeID())
		return true;
	else if (entryType == LogValues::getTypeID())
		return true;
	SimulatorLogEntry::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("entryType",ext.name) == entryType)
            return true;
	}
	return false;
}

SimulatorLogEntry::Ptr SimulatorLogEntry::Factory::makeEntry(const std::string& entryType, SimulatorLogScope* parent) {
	if (entryType == LogCollisionResult::getTypeID())
		return ownedPtr(new LogCollisionResult(parent));
	else if (entryType == LogConstraintForceTorque::getTypeID())
		return ownedPtr(new LogConstraintForceTorque(parent));
	else if (entryType == LogConstraints::getTypeID())
		return ownedPtr(new LogConstraints(parent));
	else if (entryType == LogContactForceTorque::getTypeID())
		return ownedPtr(new LogContactForceTorque(parent));
	else if (entryType == LogContactSet::getTypeID())
		return ownedPtr(new LogContactSet(parent));
	else if (entryType == LogContactVelocities::getTypeID())
		return ownedPtr(new LogContactVelocities(parent));
	else if (entryType == LogEquationSystem::getTypeID())
		return ownedPtr(new LogEquationSystem(parent));
	else if (entryType == LogPositions::getTypeID())
		return ownedPtr(new LogPositions(parent));
	else if (entryType == LogVelocities::getTypeID())
		return ownedPtr(new LogVelocities(parent));
	else if (entryType == LogMessage::getTypeID())
		return ownedPtr(new LogMessage(parent));
	else if (entryType == LogValues::getTypeID())
		return ownedPtr(new LogValues(parent));

	SimulatorLogEntry::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("entryType",ext->getName() ) == entryType){
			const rw::common::Ptr<const SimulatorLogEntry> base = ext->getObject().cast<const SimulatorLogEntry>();
			return base->createNew(parent);
		}
	}
	return NULL;
}
