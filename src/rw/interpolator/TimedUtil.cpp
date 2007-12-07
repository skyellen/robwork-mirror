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

#include "TimedUtil.hpp"

#include <rw/models/TimeMetricUtil.hpp>


using namespace rw::interpolator;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

typedef TimedUtil::TimedState TimedState;
typedef TimedUtil::TimedQ TimedQ;

std::vector<TimedQ> TimedUtil::MakeTimedQPath(
    const Q& speed,
    const std::vector<Q>& path)
{
    std::vector<TimedQ> result;
    if (path.empty()) return result;

    typedef std::vector<Q>::const_iterator I;

    I next = path.begin();
    I cur = next;

    double time = 0;
    result.push_back(TimedQ(0, *next));

    for (++next; next != path.end(); ++cur, ++next) {
        time += TimeMetricUtil::TimeDistance(*cur, *next, speed);
        result.push_back(TimedQ(time, *next));
    }

    return result;
}

TimedStatePath TimedUtil::MakeTimedStatePath(const WorkCell& speed,
											 const std::vector<State>& path)
{
    TimedStatePath result;
    if (path.empty()) 
    	return result;

    typedef std::vector<State>::const_iterator I;

    I next = path.begin();
    I cur = next;

    double time = 0;
    result.push_back(TimedState(0, *next));

    for (++next; next != path.end(); ++cur, ++next) {
        time += TimeMetricUtil::TimeDistance(*cur, *next, speed);
        result.push_back(TimedState(time, *next));
    }

    return result;
}



