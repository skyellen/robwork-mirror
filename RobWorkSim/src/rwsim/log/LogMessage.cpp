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
#include "LogMessage.hpp"

#include <boost/algorithm/string.hpp>

using namespace rw::common;
using namespace rwsim::log;

#define LINE_WIDTH 64

LogMessage::LogMessage(SimulatorLogScope* parent):
	SimulatorLogEntry(parent)
{
}

LogMessage::~LogMessage() {
}

void LogMessage::read(class InputArchive& iarchive, const std::string& id) {
	const unsigned int lines = iarchive.readUInt("MessageLines");
	std::string str;
	for (unsigned int i = 0; i < lines; i++) {
		str.append(iarchive.readString("Line"));
	}
	boost::replace_all(str, "?|!", "\n");
	_message.str(str);
	SimulatorLogEntry::read(iarchive,id);
}

void LogMessage::write(class OutputArchive& oarchive, const std::string& id) const {
	std::string str = _message.str();
	boost::replace_all(str, "\n", "?|!");
	const std::size_t fullLines = str.size()/LINE_WIDTH;
	const bool endLine = str.size()%LINE_WIDTH > 0;
	std::vector<std::string> split(fullLines + (endLine?1:0));
	oarchive.write(split.size(),"MessageLines");
	for (std::size_t i = 0; i < fullLines; i++) {
		oarchive.write(str.substr(i*LINE_WIDTH,LINE_WIDTH),"Line");
	}
	if (endLine)
		oarchive.write(str.substr(fullLines*LINE_WIDTH,std::string::npos),"Line");
	SimulatorLogEntry::write(oarchive,id);
}

std::string LogMessage::getType() const {
	return getTypeID();
}

std::list<SimulatorLogEntry::Ptr> LogMessage::getLinkedEntries() const {
	return std::list<SimulatorLogEntry::Ptr>();
}

bool LogMessage::autoLink() {
	return true;
}

SimulatorLogEntry::Ptr LogMessage::createNew(SimulatorLogScope* parent) const {
	return ownedPtr(new LogMessage(parent));
}

std::ostream& LogMessage::stream() {
	return _message;
}

std::string LogMessage::getMessage() const {
	return _message.str();
}

std::string LogMessage::getTypeID() {
	return "Message";
}
