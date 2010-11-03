#include "LuaSensor.hpp"

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

Image::Image(rw::sensor::Image::Ptr img):_image(img){}
rw::sensor::Image::Ptr Image::get(){return _image;}


