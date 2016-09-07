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

#include "LogForceTorque.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogForceTorque::LogForceTorque(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogForceTorque::~LogForceTorque() {
}

void LogForceTorque::read(InputArchive& iarchive, const std::string& id) {
	const unsigned int n = iarchive.readUInt("Wrenches");
	_forces.resize(n);
	for (unsigned int i = 0; i < n; i++) {
		iarchive.read(_forces[i].first,"Wrench6D");
		iarchive.read(_forces[i].second,"Wrench6D");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogForceTorque::write(OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_forces.size(),"Wrenches");
	std::vector<std::pair<Wrench6D<>, Wrench6D<> > >::const_iterator it;
	for (it = _forces.begin(); it != _forces.end(); it++) {
		oarchive.write(it->first,"Wrench6D");
		oarchive.write(it->second,"Wrench6D");
	}
	SimulatorLogEntry::write(oarchive,id);
}

Wrench6D<> LogForceTorque::getWrenchBodyA(std::size_t i) const {
	RW_ASSERT(i < _forces.size());
	return _forces[i].first;
}

Wrench6D<> LogForceTorque::getWrenchBodyB(std::size_t i) const {
	RW_ASSERT(i < _forces.size());
	return _forces[i].second;
}

void LogForceTorque::setWrench(std::size_t i, const Wrench6D<>& wrenchA, const Wrench6D<>& wrenchB) {
	if (i >= _forces.size()) {
		if (sizeLinkedEntry() >= 0)
			_forces.resize(std::max(i+1,(std::size_t)sizeLinkedEntry()));
		else
			_forces.resize(i+1);
	}
	_forces[i].first = wrenchA;
	_forces[i].second = wrenchB;
}
