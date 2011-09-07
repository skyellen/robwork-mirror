/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SDHDriverLua.hpp"

#include <rw/common.hpp>
#include "SDHDriver.hpp"

#include <iostream>
#include <sstream>

using namespace std;
using namespace rwlua::rwhw;
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

namespace {
    rwhw::SDHDriver *sdh = NULL;
}

rwhw::SDHDriver* rwlua::rwhw::getSDH(){
	return sdh;
}

void rwlua::rwhw::setSDH(rwhw::SDHDriver* driver){

}

