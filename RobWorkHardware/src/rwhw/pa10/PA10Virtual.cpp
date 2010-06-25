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

#include "PA10Virtual.hpp"

#include <iostream>
#include <fstream>
#include <float.h>


using namespace rwhw;
using namespace rw::math;

PA10Virtual::PA10Virtual(const Q& q):
    _q(q)
{
}

Q PA10Virtual::start(bool& b){
    b = true;
    _dqlast = Q(Q::ZeroBase(7));
   _timer.reset();


    return _q;
}


void PA10Virtual::initializeThread() {

}



Q PA10Virtual::update(const Q& dq) {
    _timer.pause();
    double dt = _timer.getTime();
    _timer.reset();
    dt = 0.01;

    _q += dt*(dq+_dqlast)/2.0;
   // _q += dt*(_dqlast)/2.0;
    //_q += dt*(dq)/2.0;
    _dqlast = dq;

    return _q;
}



void PA10Virtual::stop(){
    _timer.reset();
}

