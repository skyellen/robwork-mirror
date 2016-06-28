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


#include "TimedUtil.hpp"
#include "TimeMetricUtil.hpp"

#include <rw/models/Device.hpp>

#include <boost/foreach.hpp>

using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

typedef Timed<State> TimedState;
typedef Timed<Q> TimedQ;

TimedQPath TimedUtil::makeTimedQPath(
    const Q& speed,
    const QPath& path,
    double offset)
{
    TimedQPath result;
    if (path.empty()) return result;

    typedef QPath::const_iterator I;

    I next = path.begin();
    I cur = next;

    double time = offset;
    result.push_back(TimedQ(time, *next));

    for (++next; next != path.end(); ++cur, ++next) {
        time += TimeMetricUtil::timeDistance(*cur, *next, speed);
        result.push_back(TimedQ(time, *next));
    }

    return result;
}

TimedQPath TimedUtil::makeTimedQPath(
    const Device& device, const QPath& path, double offset)
{
    return makeTimedQPath(device.getVelocityLimits(), path, offset);
}

TimedStatePath TimedUtil::makeTimedStatePath(
    const WorkCell& speed,
    const StatePath& path,
	double offset)
{
    TimedStatePath result;
    if (path.empty())
    	return result;

    typedef StatePath::const_iterator I;

    I next = path.begin();
    I cur = next;

    double time = offset;
    result.push_back(TimedState(0, *next));

    for (++next; next != path.end(); ++cur, ++next) {
        time += TimeMetricUtil::timeDistance(*cur, *next, speed);
        result.push_back(TimedState(time, *next));
    }

    return result;
}

TimedStatePath TimedUtil::makeTimedStatePath(
    const Device& device,
    const QPath& path,
    const State& common_state,
	double offset)
{
    State state = common_state;
    const TimedQPath qts = makeTimedQPath(device.getVelocityLimits(), path, offset);

    TimedStatePath result;
    typedef Timed<Q> TimedQ;
    BOOST_FOREACH(const TimedQ& qt, qts) {
        device.setQ(qt.getValue(), state);
        result.push_back(
            makeTimed(
                qt.getTime(),
                state));
    }
    return result;
}
