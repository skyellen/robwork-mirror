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


#include "LogFileWriter.hpp"

#include "macros.hpp"
#include <fstream>
#include <iomanip>

using namespace rw::common;

LogFileWriter::LogFileWriter(const std::string& filename):
	_tabLevel(0)
{
    try {
        _stream.open(filename.c_str());
    } catch (const std::exception& exp) {
        RW_THROW("Unable to open log file with message: "<<exp.what());
    }
    if (!_stream.is_open()) {
        RW_THROW("Unable to open log file with message!");
    }
}

LogFileWriter::~LogFileWriter()
{
    flush();
    _stream.close();
}


void LogFileWriter::doWrite(const std::string& str)
{
	_stream << std::setw(4*_tabLevel)<<std::setfill(' ')<<"";
	_stream << str; 
	_stream.flush();
}

void LogFileWriter::doFlush()
{
    _stream.flush();
}

void LogFileWriter::doSetTabLevel(int tabLevel) {
	_tabLevel = tabLevel;
}
