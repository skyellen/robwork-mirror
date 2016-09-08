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

#include "LogEquationSystem.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

using namespace rw::common;
using namespace rwsim::log;

LogEquationSystem::LogEquationSystem(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogEquationSystem::~LogEquationSystem() {
}

void LogEquationSystem::read(InputArchive& iarchive, const std::string& id) {
	iarchive.read(_A,"A");
	iarchive.read(_b,"b");
	iarchive.read(_x,"x");
	SimulatorLogEntry::read(iarchive,id);
}

void LogEquationSystem::write(OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_A,"A");
	oarchive.write(_b,"b");
	oarchive.write(_x,"x");
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogEquationSystem::getType() const {
	return getTypeID();
}

bool LogEquationSystem::operator==(const SimulatorLog &b) const {
	if (const LogEquationSystem* const entry = dynamic_cast<const LogEquationSystem*>(&b)) {
		if (_A != entry->_A)
			return false;
		if (_x != entry->_x)
			return false;
		if (_b != entry->_b)
			return false;
	}
	return SimulatorLogEntry::operator==(b);
}

std::list<SimulatorLogEntry::Ptr> LogEquationSystem::getLinkedEntries() const {
	return std::list<SimulatorLogEntry::Ptr>();
}

bool LogEquationSystem::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogEquationSystem::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogEquationSystem(parent));
}

std::string LogEquationSystem::getTypeID() {
	return "LogEquationSystem";
}

void LogEquationSystem::set(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
	_A = A;
	_b = b;
}

void LogEquationSystem::setSolution(const Eigen::VectorXd& x) {
	_x = x;
}

const Eigen::MatrixXd& LogEquationSystem::A() const {
	return _A;
}

const Eigen::VectorXd& LogEquationSystem::b() const {
	return _b;
}

const Eigen::VectorXd& LogEquationSystem::x() const {
	return _x;
}
