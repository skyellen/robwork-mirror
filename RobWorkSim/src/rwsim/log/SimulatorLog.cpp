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

#include "SimulatorLog.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include <boost/algorithm/string.hpp>

using namespace rw::common;
using namespace rwsim::log;

#define LINE_WIDTH 64

SimulatorLog::SimulatorLog(SimulatorLogScope* parent):
	_parent(parent)
{
}

SimulatorLog::~SimulatorLog() {
}

void SimulatorLog::read(class InputArchive& iarchive, const std::string& id) {
	_filename = "";
	const unsigned int lines = iarchive.readUInt("FilenameLines");
	for (unsigned int i = 0; i < lines; i++) {
		_filename.append(iarchive.readString("Filename"));
	}
	boost::replace_all(_filename, "?|!", "\n");
	_description = iarchive.readString("Description");
}

void SimulatorLog::write(class OutputArchive& oarchive, const std::string& id) const {
	std::string str = _filename;
	boost::replace_all(str, "\n", "?|!");
	const std::size_t fullLines = str.size()/LINE_WIDTH;
	const bool endLine = str.size()%LINE_WIDTH > 0;
	std::vector<std::string> split(fullLines + (endLine?1:0));
	oarchive.write(split.size(),"FilenameLines");
	for (std::size_t i = 0; i < fullLines; i++) {
		oarchive.write(str.substr(i*LINE_WIDTH,LINE_WIDTH),"Filename");
	}
	if (endLine)
		oarchive.write(str.substr(fullLines*LINE_WIDTH,std::string::npos),"Filename");
	oarchive.write(_description,"Description");
}

SimulatorLogScope* SimulatorLog::getParent() const {
	return _parent;
}

bool SimulatorLog::operator==(const SimulatorLog &b) const {
	if (_filename != b._filename)
		return false;
	if (_description != b._description)
		return false;
	return true;
}

bool SimulatorLog::operator!=(const SimulatorLog &b) const {
	return !operator==(b);
}

std::string SimulatorLog::getFilename() const {
	return _filename;
}

void SimulatorLog::setFilename(const std::string& file) {
	_filename = file;
}

void SimulatorLog::setFilename(const char* file) {
	_filename = file;
}

std::string SimulatorLog::getDescription() const {
	return _description;
}

void SimulatorLog::setDescription(const std::string& description) {
	_description = description;
}
