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
#include "SimulatorLogEntry.hpp"

#include "SimulatorStatistics.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include <boost/foreach.hpp>
#include "LogStep.hpp"

using namespace rw::common;
using namespace rwsim::log;

SimulatorLogScope::SimulatorLogScope(SimulatorLogScope* parent):
	SimulatorLog(parent),
	_line(std::make_pair<int,int>(-1,-1)) {
}

SimulatorLogScope::~SimulatorLogScope() {
}

void SimulatorLogScope::read(class InputArchive& iarchive, const std::string& id) {
	_children.clear();
	_line.first = iarchive.readInt("LineFirst");
	_line.second = iarchive.readInt("LineSecond");
	unsigned int children = iarchive.readUInt("Children");
	_children.resize(children);
	for (unsigned int i = 0; i < children; i++) {
		const std::string type = iarchive.readString("ChildType");
		if (type == "Scope")
			_children[i] = ownedPtr(new SimulatorLogScope(this));
		else if (type == "Step")
			_children[i] = ownedPtr(new LogStep(this));
		else {
			if (!SimulatorLogEntry::Factory::hasEntryType(type))
				RW_THROW("Could not read entry of type \"" << type << "\" - type could not be found in factory.");
			const SimulatorLogEntry::Ptr entry = SimulatorLogEntry::Factory::makeEntry(type,this);
			_children[i] = entry;
			entry->autoLink();
		}
		_children[i]->read(iarchive,id);
	}
	SimulatorLog::read(iarchive,id);
	_statistics = NULL;
}

void SimulatorLogScope::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_line.first,"LineFirst");
	oarchive.write(_line.second,"LineSecond");
	oarchive.write(_children.size(),"Children");
	BOOST_FOREACH(const SimulatorLog::Ptr child, _children) {
		oarchive.write(child->getType(),"ChildType");
		child->write(oarchive,"");
	}
	SimulatorLog::write(oarchive,id);
}

std::size_t SimulatorLogScope::children() const {
	return _children.size();
}

std::string SimulatorLogScope::getType() const {
	return "Scope";
}

std::vector<SimulatorLog::Ptr> SimulatorLogScope::getChildren() const {
	return _children;
}

SimulatorLog::Ptr SimulatorLogScope::getChild(std::size_t id) const {
	RW_ASSERT(id < children());
	return _children[id];
}

std::size_t SimulatorLogScope::indexOf(const SimulatorLog* child) const {
	std::size_t i = 0;
	BOOST_FOREACH(const SimulatorLog::Ptr c, _children) {
		if (c.get() == child)
			return i;
		i++;
	}
	return _children.size();
}

void SimulatorLogScope::appendChild(SimulatorLog::Ptr child) {
	_children.push_back(child);
	_statistics = NULL;
}

rw::common::Ptr<const SimulatorStatistics> SimulatorLogScope::getStatistics() {
	if (_statistics == NULL) {
		_statistics = ownedPtr(new SimulatorStatistics(this));
	}
	_statistics->update();
	if (_statistics->hasData() == 0)
		return 0;
	else
		return _statistics;
}

int SimulatorLogScope::lineBegin() const {
	return _line.first;
}

int SimulatorLogScope::lineEnd() const {
	return _line.second;
}

void SimulatorLogScope::setLineBegin(int line) {
	_line.first = line;
}

void SimulatorLogScope::setLineEnd(int line) {
	_line.second = line;
}
