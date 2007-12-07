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

#include "PathPlanner.hpp"

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Math.hpp>

using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;

PathPlanner::~PathPlanner()
{}

PropertyMap& PathPlanner::getProperties()
{
    return _properties;
}

const PropertyMap& PathPlanner::getProperties() const
{
    return _properties;
}
