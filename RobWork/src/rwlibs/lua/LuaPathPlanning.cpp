#include "LuaTrajectory.hpp"

using namespace rwlua::rw;
#include <iostream>
using namespace std;
#include <sstream>

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


