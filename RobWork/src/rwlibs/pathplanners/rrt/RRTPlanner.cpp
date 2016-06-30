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


#include "RRTPlanner.hpp"

#include "RRTQToQPlanner.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/QSampler.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;
using namespace rw::common;

QToQPlanner::Ptr RRTPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
	QSampler::Ptr sampler,
	QMetric::Ptr metric,
    double extend,
    PlannerType type)
{
    switch (type) {
    case RRTBasic:
        return RRTQToQPlanner::makeBasic(constraint, sampler, metric, extend);
    case RRTConnect:
        return RRTQToQPlanner::makeConnect(constraint, sampler, metric, extend);
    case RRTBidirectional:
        return RRTQToQPlanner::makeBidirectional(
            constraint, sampler, metric, extend);
    case RRTBalancedBidirectional:
        return RRTQToQPlanner::makeBalancedBidirectional(
            constraint, sampler, metric, extend);
    }
    RW_ASSERT(0);
    return 0;
}

QToQPlanner::Ptr RRTPlanner::makeQToQPlanner(
    const PlannerConstraint& constraint,
	Device::Ptr device,
    PlannerType type)
{
    const double extend = 0.05;

    return makeQToQPlanner(
        constraint,
        QSampler::makeUniform(device),
        PlannerUtil::normalizingInfinityMetric(device->getBounds()),
        extend,
        type);
}
