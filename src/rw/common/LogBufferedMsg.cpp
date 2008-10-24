/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "LogBufferedMsg.hpp"

#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::common;

LogBufferedMsg::LogBufferedMsg(std::ostream* stream):
    _stream(stream)
{}

LogBufferedMsg::~LogBufferedMsg()
{
    flush();
}

void LogBufferedMsg::write(const std::string& msg)
{
    _buffer.push_back(msg);
}

void LogBufferedMsg::flush()
{
    BOOST_FOREACH(const std::string& str, _buffer) {
        *_stream << str;
    }
    _stream->flush();
    _buffer.clear();
}
