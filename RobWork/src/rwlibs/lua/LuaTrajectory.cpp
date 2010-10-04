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
/*
QPath::QPath(){}
QPath::QPath(const rw::trajectory::QPath& path):
		rw::trajectory::QPath(path)
{}
*/
////////// TimedQPath
/*
TimedQ::TimedQ(rw::trajectory::Timed<rw::math::Q>& value):_value(value.getTime(), value.getValue()){}
TimedQ::TimedQ(double time, Q q):_value(time,q){}

double& TimedQ::getTime(){ return _value.getTime(); }
Q& TimedQ::getQ(){ return _value.getValue(); }

//rw::trajectory::Timed<rw::math::Q>& TimedQ::get(){ return _value; }
std::string TimedQ::__tostring() const{ return ""; };


TimedQPath::TimedQPath(){}
int TimedQPath::size(){ return _path.size(); }
TimedQ& TimedQPath::operator[] (int index){ return _path[index]; }

*/

//void TimedQPath::add(double time, rwlibs::lua::Q value){
//    this->push_back(rw::trajectory::Timed<rw::trajectory::Q>(time, value) );
//}


//////////// StatePath
/*
StatePath::StatePath(){}
StatePath::StatePath(const rw::trajectory::StatePath& path):
		rw::trajectory::StatePath(path)
{}

TimedStatePath::TimedStatePath(const rw::trajectory::TimedStatePath& path):
		rw::trajectory::TimedStatePath(path)
{}*/


