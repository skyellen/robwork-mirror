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


#include "Joint.hpp"

#include <cfloat>


using namespace rw::models;
using namespace rw::math;

Joint::Joint(const std::string& name, size_t dof) :
    Frame(dof, name),
    _bounds(Q(dof), Q(dof)),
    _maxVelocity(Q(dof)),
    _maxAcceleration(Q(dof))
{
    for (size_t i = 0; i<dof; i++) {
        _bounds.first(i) = -DBL_MAX;
        _bounds.second(i) = DBL_MAX;
        _maxVelocity(i) = 1;
        _maxAcceleration(i) = 1;
    }
}


Joint::Joint(const std::string& name, size_t dof, size_t stateSize):
        Frame(dof, name),
        _bounds(Q(dof), Q(dof)),
        _maxVelocity(Q(dof)),
        _maxAcceleration(Q(dof))
{
    for (size_t i = 0; i<dof; i++) {
        _bounds.first(i) = -DBL_MAX;
        _bounds.second(i) = DBL_MAX;
        _maxVelocity(i) = 1;
        _maxAcceleration(i) = 1;
    }
}
