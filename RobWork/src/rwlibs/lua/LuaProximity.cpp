#include "LuaProximity.hpp"

using namespace rwlibs::lua::proximity;
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

CollisionSetup::CollisionSetup(const rw::proximity::CollisionSetup& setup):
		rw::proximity::CollisionSetup(setup)
{}
