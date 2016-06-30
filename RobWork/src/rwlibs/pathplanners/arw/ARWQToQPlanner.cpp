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


#include "ARWQToQPlanner.hpp"
#include <rw/common/macros.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <boost/foreach.hpp>
#include <rw/trajectory/Path.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

ARWQToQPlanner::ARWQToQPlanner(
    const rw::pathplanning::PlannerConstraint& constraint,
	ARWExpand::Ptr expand,
	rw::math::QMetric::Ptr metric,
    double nearDistance)
    :
    _constraint(constraint),
    _expand(expand),
    _metric(metric),
    _nearDistance(nearDistance)
{
    RW_ASSERT(nearDistance >= 0);
}

bool ARWQToQPlanner::nearGoal(const Q& q, const Q& goal) const
{
    return _metric->distance(q, goal) < _nearDistance;
}

bool ARWQToQPlanner::planPathStep(
    ARWExpand& expand,
    const std::vector<Q>& goals) const
{
    if (expand.expand()) {
        const Q& pos = expand.getPath().back();

        BOOST_FOREACH(const Q& goal, goals) {
            if (
                nearGoal(pos, goal) &&
                !PlannerUtil::inCollision(
                    _constraint, pos, goal, false, false))
            {
                return true;
            }
        }
    }

    return false;
}

bool ARWQToQPlanner::doQuery(
    const Q& from,
    const Q& to,
    QPath& result,
    const StopCriteria& stop)
{
	ARWExpand::Ptr forwardPath = _expand->duplicate(from);
	ARWExpand::Ptr backwardPath = _expand->duplicate(to);

    std::vector<Q> forwardGoals; forwardGoals.push_back(to);
    std::vector<Q> backwardGoals; backwardGoals.push_back(from);

    while (true) {

        if (stop.stop()) break;

        forwardGoals.push_back(backwardPath->getPath().back());
        const bool okF = planPathStep(*forwardPath, forwardGoals);
        forwardGoals.pop_back();

        if (okF) {
            result.insert(
                result.end(),
                forwardPath->getPath().begin(),
                forwardPath->getPath().end());
            return true;
        }

        if (stop.stop()) break;

        backwardGoals.push_back(forwardPath->getPath().back());
        const bool okB = planPathStep(*backwardPath, backwardGoals);
        backwardGoals.pop_back();

        if (okB) {
            result.insert(
                result.end(),
                backwardPath->getPath().rbegin(),
                backwardPath->getPath().rend());
            return true;
        }
    }

    return false;
}
