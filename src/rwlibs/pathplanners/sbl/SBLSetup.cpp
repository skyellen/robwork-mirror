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

#include "SBLSetup.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;

SBLSetup SBLSetup::make(
    const PlannerConstraint& constraint,
    QExpandPtr expansion,
    QMetricPtr metric,
    double connectRadius)
{
    return SBLSetup(SBLOptions(constraint, expansion, metric, connectRadius));
}

SBLSetup SBLSetup::make(
    const PlannerConstraint& constraint,
    DevicePtr device,
    double expandRadius,
    double connectRadius)
{
    if (expandRadius < 0) expandRadius = 0.25;
    if (connectRadius < 0) connectRadius = 0.5;

    return make(
        constraint,
        QExpand::makeShrinkingUniformBox(
            constraint.getQConstraintPtr(),
            device->getBounds(),
            2 * expandRadius),
        PlannerUtil::normalizingInfinityMetric(
            device->getBounds()),
        connectRadius);
}
