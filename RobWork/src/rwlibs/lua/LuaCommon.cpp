#include "LuaCommon.hpp"

using namespace rwlibs::lua::common;
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

void rwlibs::lua::common::info(const std::string& msg){
	rw::common::Log::infoLog() << msg;
}

void rwlibs::lua::common::debug(const std::string& msg){
	rw::common::Log::debugLog() << msg;
}

void rwlibs::lua::common::warn(const std::string& msg){
	rw::common::Log::warningLog() << msg;
}

void rwlibs::lua::common::error(const std::string& msg){
	rw::common::Log::errorLog() << msg;
}
