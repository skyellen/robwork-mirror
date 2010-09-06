#include "LuaTrajectory.hpp"

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


QPath::QPath(const rw::trajectory::QPath& path):
		rw::trajectory::QPath(path)
{}

StatePath::StatePath(const rw::trajectory::StatePath& path):
		rw::trajectory::StatePath(path)
{}

TimedStatePath::TimedStatePath(const rw::trajectory::TimedStatePath& path):
		rw::trajectory::TimedStatePath(path)
{}


