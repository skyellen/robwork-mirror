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


#include "LogBufferedMsg.hpp"

#include <boost/foreach.hpp>
#include <iomanip>

using namespace rw::common;

LogBufferedMsg::LogBufferedMsg(std::ostream* stream):
    _stream(stream),
	_tabLevel(0)
{}

LogBufferedMsg::~LogBufferedMsg()
{
    flush();
}

void LogBufferedMsg::write(const std::string& msg)
{
	_buffer.push_back(std::make_pair(msg, _tabLevel));
}

void LogBufferedMsg::flush()
{
	typedef std::pair<std::string, int> StringIntPair;
	BOOST_FOREACH(const StringIntPair& pair, _buffer) {
		*_stream << std::setw(pair.second)<<std::setfill(' ');
        *_stream << pair.first;
    }
    _stream->flush();
    _buffer.clear();
}

void LogBufferedMsg::setTabLevel(int tablevel) {
	_tabLevel = tablevel;
}
