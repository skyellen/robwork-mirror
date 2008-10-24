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

#include "ARWPlanner.hpp"
#include "ARWQToQPlanner.hpp"
#include <rw/math/Math.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;

QToQPlannerPtr
ARWPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
    ARWExpandPtr expand,
    QMetricPtr metric,
    double nearDistance)
{
    return ownedPtr(
        new ARWQToQPlanner(
            constraint,
            expand,
            metric,
            nearDistance));
}

QToQPlannerPtr
ARWPlanner::makeQToQPlanner(
    const rw::pathplanning::PlannerConstraint& constraint,
    rw::models::DevicePtr device,
    rw::math::QMetricPtr metric,
    double nearDistance,
    int historySize)
{
    if (!metric) {
        metric = PlannerUtil::normalizingInfinityMetric(device->getBounds());
        nearDistance = 0.5;
    }

    if (historySize < 0) historySize = 20;

    return makeQToQPlanner(
        constraint,
        ARWExpand::make(
            device->getBounds(),
            constraint,
            Q(),
            historySize),
        metric,
        nearDistance);
}
