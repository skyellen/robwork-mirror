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
    std::cout<<"Push Message:"<<msg<<std::endl;
    _buffer.push_back(msg);
}

void LogBufferedMsg::flush()
{
    BOOST_FOREACH(const std::string& str, _buffer) {
        std::cout<<"Flush:"<<str<<std::endl;
        *_stream << str;
    }
    _stream->flush();
    _buffer.clear();
}
