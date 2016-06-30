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


#include "ARWPlanner.hpp"
#include "ARWQToQPlanner.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;

QToQPlanner::Ptr ARWPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
	ARWExpand::Ptr expand,
	QMetric::Ptr metric,
    double nearDistance)
{
    return ownedPtr(
        new ARWQToQPlanner(
            constraint,
            expand,
            metric,
            nearDistance));
}

QToQPlanner::Ptr ARWPlanner::makeQToQPlanner(
    const rw::pathplanning::PlannerConstraint& constraint,
	rw::models::Device::Ptr device,
	rw::math::QMetric::Ptr metric,
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
