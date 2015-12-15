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
#include "LogValues.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsim::log;

LogValues::LogValues(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogValues::~LogValues() {}

void LogValues::read(class InputArchive& iarchive, const std::string& id) {
	std::size_t n;
	n = iarchive.readUInt("Values");
	_values.resize(n);
	for (std::size_t i = 0; i < n; i++) {
		iarchive.read(_values[i],"Value");
	}
	n = iarchive.readUInt("Labels");
	_labels.resize(n);
	for (std::size_t i = 0; i < n; i++) {
		_labels[i] = iarchive.readString("Label");
	}
	SimulatorLogEntry::read(iarchive,id);
}

void LogValues::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_values.size(),"Values");
	BOOST_FOREACH(const double value, _values) {
		oarchive.write(value,"Value");
	}
	oarchive.write(_labels.size(),"Labels");
	BOOST_FOREACH(const std::string& label, _labels) {
		oarchive.write(label,"Label");
	}
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogValues::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogValues::getLinkedEntries() const {
	return std::list<SimulatorLogEntry::Ptr>(0);
}

bool LogValues::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogValues::createNew(SimulatorLogScope* parent) const {
	return rw::common::ownedPtr(new LogValues(parent));
}

void LogValues::setData(const std::vector<std::string>& labels, const std::vector<double>& values) {
	RW_ASSERT(labels.size() == values.size());
	_values = values;
	_labels = labels;
}

std::size_t LogValues::size() const {
	RW_ASSERT(_values.size() == _labels.size());
	return _values.size();
}

const std::vector<std::string>& LogValues::getLabels() const {
	return _labels;
}

std::string LogValues::getLabel(std::size_t i) const {
	return _labels[i];
}

const std::vector<double>& LogValues::getValues() const {
	return _values;
}

double LogValues::getValue(std::size_t i) const {
	return _values[i];
}

std::string LogValues::getTypeID() {
	return "Values";
}
