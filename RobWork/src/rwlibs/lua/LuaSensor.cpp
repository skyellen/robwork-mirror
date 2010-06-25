#include "LuaSensor.hpp"

using namespace rwlibs::lua::sensor;
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

Image::Image(rw::sensor::ImagePtr img):_image(img){}
rw::sensor::ImagePtr Image::get(){return _image;}


