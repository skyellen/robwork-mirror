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


#include "TimeMetricUtil.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;


//----------------------------------------------------------------------
// Time distances

double TimeMetricUtil::timeDistance(
    const Q& from,
    const Q& to,
    const Q& velocity)
{
    RW_ASSERT(from.size() == to.size());
    RW_ASSERT(velocity.size() <= from.size());
    double result = 0;
    for (size_t i = 0; i < velocity.size(); i++) {
        result = std::max(result, fabs((from[i] - to[i]) / velocity[i]));
    }
    return result;
}

double TimeMetricUtil::timeDistance(
    const Q& from,
    const Q& to,
    const Device& device)
{
    return timeDistance(from, to, device.getVelocityLimits());
}

double TimeMetricUtil::timeDistance(
    const State& from,
    const State& to,
    const Device& device)
{
    return timeDistance(device.getQ(from), device.getQ(to), device.getVelocityLimits());
}

double TimeMetricUtil::timeDistance(
    const State& from,
    const State& to,
    const WorkCell& workcell)
{
    // We simply compute the time distance for each device and choose the
    // maximum value.

    const int len = (int)workcell.getDevices().size();
    Q device_times(len);

    for (int i = 0; i < len; i++) {
        const Device& device = *workcell.getDevices()[i];
        device_times[i] = timeDistance(from, to, device);
    }

    return MetricUtil::normInf(device_times);
}
