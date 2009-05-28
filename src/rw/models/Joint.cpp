/*********************************************************************
 * RobWork Version 0.3
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
