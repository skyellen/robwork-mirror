#include "LogBufferedMsg.hpp"

#include <boost/foreach.hpp>
#include <iostream>

using namespace rw::sandbox;

LogBufferedMsg::LogBufferedMsg(std::ostream& stream):
    _stream(stream)
{
}

LogBufferedMsg::~LogBufferedMsg()
{
    flush();
}

void LogBufferedMsg::write(const std::string& msg) {
    _buffer.push_back(msg);
}

void LogBufferedMsg::flush() {
    std::cout<<"Prints "<<_buffer.size()<<std::endl;
    BOOST_FOREACH(std::string str, _buffer) {
        std::cout<<"str = "<<str<<std::endl;
        _stream<<str;
    }
    _stream.flush();

    _buffer.clear();
}
