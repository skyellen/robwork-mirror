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

#include "ARWQToQPlanner.hpp"
#include <rw/common/macros.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/models/Models.hpp>
#include <boost/foreach.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

ARWQToQPlanner::ARWQToQPlanner(
    const rw::pathplanning::PlannerConstraint& constraint,
    ARWExpandPtr expand,
    rw::math::QMetricPtr metric,
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
    ARWExpandPtr forwardPath = _expand->duplicate(from);
    ARWExpandPtr backwardPath = _expand->duplicate(to);

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
