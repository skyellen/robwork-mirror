/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PA10Virtual.hpp"

#include <iostream>
#include <fstream>
#include <float.h>


using namespace rwlibs::devices;
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

