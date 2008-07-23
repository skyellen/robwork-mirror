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

#include "SBLOptions.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;

SBLOptions::SBLOptions(
    const PlannerConstraint& constraint,
    QExpandPtr expansion,
    MetricPtr metric,
    double connectRadius)
    :
    constraint(constraint),
    expansion(expansion),
    metric(metric),
    connectRadius(connectRadius)
{
    resetCount = 200;
    rootSampleInterval = 25;
    nodesPerCell = 10;
    nearNodeSelection = NearestNode;
}
