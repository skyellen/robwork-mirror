#include "LuaCommon.hpp"

#include <rw/common/TimerUtil.hpp>

using namespace rwlua::rw;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const ::rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

void rwlua::rw::info(const std::string& msg){
	::rw::common::Log::infoLog() << msg;
}

void rwlua::rw::debug(const std::string& msg){
	::rw::common::Log::debugLog() << msg;
}

void rwlua::rw::warn(const std::string& msg){
	::rw::common::Log::warningLog() << msg;
}

void rwlua::rw::error(const std::string& msg){
	::rw::common::Log::errorLog() << msg;
}

void rwlua::rw::sleep(double t){
    ::rw::common::TimerUtil::sleepMs( (int) (t*1000) );
}

void rwlua::rw::lualog(const std::string& msg){
    ::rw::common::Log::log().write(::rw::common::Log::User1, msg);
}

void rwlua::rw::setLualog(::rw::common::LogWriter::Ptr writer){
    ::rw::common::Log::log().setWriter(::rw::common::Log::User1, writer);
}
