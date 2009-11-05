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


#include "PointTimeIndexFactory.hpp"
#include "PointTimeIndex.hpp"
#include <rw/math/MetricUtil.hpp>
#include "TimedUtil.hpp"

#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

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
    return make(TimedUtil::makeTimedQPath(speed, path));
}

std::auto_ptr<PointTimeIndex> NS::make(
    const WorkCell& workcell,
    const std::vector<State>& path)
{
    return make(TimedUtil::makeTimedStatePath(workcell, path));
}
