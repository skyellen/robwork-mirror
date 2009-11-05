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

#include <rw/models/TimeMetricUtil.hpp>


using namespace rw::interpolator;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

typedef TimedUtil::TimedState TimedState;
typedef TimedUtil::TimedQ TimedQ;

std::vector<TimedQ> TimedUtil::makeTimedQPath(
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
        time += TimeMetricUtil::timeDistance(*cur, *next, speed);
        result.push_back(TimedQ(time, *next));
    }

    return result;
}

TimedStatePath TimedUtil::makeTimedStatePath(
    const WorkCell& speed,
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
        time += TimeMetricUtil::timeDistance(*cur, *next, speed);
        result.push_back(TimedState(time, *next));
    }

    return result;
}



