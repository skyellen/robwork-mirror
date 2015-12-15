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

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>
#include "LogPositions.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogPositions::LogPositions(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogPositions::~LogPositions() {
}

void LogPositions::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("Positions");
	for (unsigned int i = 0; i < n; i++) {
		const std::string name = iarchive.readString("Name");
		iarchive.read(_positions[name],"Transform3D");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogPositions::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_positions.size(),"Positions");
	std::map<std::string,Transform3D<> >::const_iterator it;
	for (it = _positions.begin(); it != _positions.end(); it++) {
		oarchive.write(it->first,"Name");
		oarchive.write(it->second,"Transform3D");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogPositions::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogPositions::getLinkedEntries() const {
	return std::list<SimulatorLogEntry::Ptr>();
}

bool LogPositions::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogPositions::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogPositions(parent));
}

const std::map<std::string, Transform3D<> >& LogPositions::getPositions() const {
	return _positions;
}

const Transform3D<>& LogPositions::getPosition(const std::string& name) const {
	const std::map<std::string, Transform3D<> >::const_iterator it = _positions.find(name);
	RW_ASSERT(it != _positions.end());
	return it->second;
}

bool LogPositions::has(const std::string& name) const {
	const std::map<std::string, Transform3D<> >::const_iterator it = _positions.find(name);
	return it != _positions.end();
}

void LogPositions::setPositions(const std::map<std::string, Transform3D<> >& positions) {
	_positions = positions;
}

std::size_t LogPositions::size() const {
	return _positions.size();
}

std::string LogPositions::getTypeID() {
	return "Positions";
}
