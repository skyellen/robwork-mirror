#include "LuaCommon.hpp"

#include <rw/common/TimerUtil.hpp>

using namespace rwlibs::lua;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const rw::common::Exception& e)
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

void rwlibs::lua::info(const std::string& msg){
	rw::common::Log::infoLog() << msg;
}

void rwlibs::lua::debug(const std::string& msg){
	rw::common::Log::debugLog() << msg;
}

void rwlibs::lua::warn(const std::string& msg){
	rw::common::Log::warningLog() << msg;
}

void rwlibs::lua::error(const std::string& msg){
	rw::common::Log::errorLog() << msg;
}

void rwlibs::lua::sleep(double t){
    rw::common::TimerUtil::sleepMs( (int) (t*1000) );
}
