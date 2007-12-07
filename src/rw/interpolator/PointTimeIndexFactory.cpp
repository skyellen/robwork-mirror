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

#include "PointTimeIndexFactory.hpp"
#include "PointTimeIndex.hpp"
#include <rw/math/MetricUtil.hpp>
#include "TimedUtil.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/DeviceModel.hpp>

#define NS PointTimeIndexFactory

using namespace rw::interpolator;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

std::auto_ptr<PointTimeIndex> NS::makeIndex(const std::vector<double>& timeSteps)
{
    typedef std::auto_ptr<PointTimeIndex> T;
    return T(new PointTimeIndex(timeSteps));
}

std::auto_ptr<PointTimeIndex> NS::make(
    const Q& speed,
    const std::vector<Q>& path)
{
    return make(TimedUtil::MakeTimedQPath(speed, path));
}

std::auto_ptr<PointTimeIndex> NS::make(
    const WorkCell& workcell,
    const std::vector<State>& path)
{
    return make(TimedUtil::MakeTimedStatePath(workcell, path));
}
