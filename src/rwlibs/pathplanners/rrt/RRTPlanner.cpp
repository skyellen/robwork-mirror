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

#include "RRTPlanner.hpp"

#include "RRTQToQPlanner.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;
using namespace rw::common;

QToQPlannerPtr RRTPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
{
    return ownedPtr(new RRTQToQPlanner(constraint, sampler, metric, extend));
}

QToQPlannerPtr RRTPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
    DevicePtr device)
{
    const double extend = 0.05;

    return makeQToQPlanner(
        constraint,
        QSampler::makeUniform(device),
        PlannerUtil::normalizingInfinityMetric(device->getBounds()),
        extend);
}
