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
