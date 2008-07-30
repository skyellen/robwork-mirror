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
#include "Trajectory.hpp"

#include <assert.h>

using namespace rw::interpolator;
using namespace rw::math;

// TODO: this is not efficient and should probably be avoided.
Q Trajectory::getX(double d)
{
    if(_interpolators.empty())
        return Q(Q::ZeroBase(0));
    if(d<0)
        return _interpolators[0]->getX(0.0);

    double curr_length=0;
    for(unsigned int i=0;i<_interpolators.size();i++){
        if(d<curr_length+_interpolators[i]->getLength()){
            return _interpolators[i]->getX(d-curr_length);
        }
        curr_length += _interpolators[i]->getLength();
    }
    Interpolator *last = _interpolators[_interpolators.size()-1];
    return last->getX(last->getLength());
}
