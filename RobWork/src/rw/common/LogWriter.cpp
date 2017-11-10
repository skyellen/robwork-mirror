/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "LogWriter.hpp"

#include <sstream>


using namespace rw::common;

void LogWriter::write(const std::string& message) {
	boost::mutex::scoped_lock lock(_mutex);
	doWrite(message);
}

void LogWriter::setTabLevel(int tabLevel)
{
	boost::mutex::scoped_lock lock(_mutex);
	doSetTabLevel(tabLevel);
}

void LogWriter::flush() {
	boost::mutex::scoped_lock lock(_mutex);
	doFlush();
}

void LogWriter::write(const Message& msg)
{
    std::ostringstream buf;
    buf << msg;
    write(buf.str());
}

void LogWriter::writeln(const std::string& str)
{
    write(str + '\n');
}

LogWriter& LogWriter::operator<<(std::ostream& (*pf)(std::ostream&)){
	std::ostringstream buf;
	buf << pf;
	write(buf.str());
    return *this;
}

LogWriter::~LogWriter()
{}
