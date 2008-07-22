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

#include "QToQPlanner.hpp"

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Math.hpp>

using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;

namespace
{
    class RegionPlanner : public QToQPlanner
    {
	public:
        RegionPlanner(QToQSamplerPlannerPtr planner) :
            _planner(planner)
        {}

    private:
        bool doQuery(
            const rw::math::Q& from,
            const rw::math::Q& to,
            Path& path,
            const StopCriteria& stop)
        {
            return _planner->query(
                from,
                *QSampler::makeSingle(to),
                path,
                stop);
        }

    private:
        QToQSamplerPlannerPtr _planner;
    };

    class EdgePlanner : public QToQPlanner
    {
    public:
        EdgePlanner(
            QConstraintPtr constraint,
            QEdgeConstraintPtr edge)
            :
            _constraint(constraint),
            _edge(edge)
        {}

    private:
        bool doQuery(
            const rw::math::Q& from,
            const rw::math::Q& to,
            Path& path,
            const StopCriteria& stop)
        {
            if (_constraint->inCollision(from) ||
                _constraint->inCollision(to) ||
                _edge->inCollision(from, to))
            {
                return false;
            } else {
                path.push_back(from);
                path.push_back(to);
                return true;
            }
        }

    private:
        QConstraintPtr _constraint;
        QEdgeConstraintPtr _edge;
    };
}

std::auto_ptr<QToQPlanner> QToQPlanner::make(QToQSamplerPlannerPtr planner)
{
    typedef std::auto_ptr<QToQPlanner> T;
    return T(new RegionPlanner(planner));
}

std::auto_ptr<QToQPlanner> QToQPlanner::make(
    QConstraintPtr constraint,
    QEdgeConstraintPtr planner)
{
    typedef std::auto_ptr<QToQPlanner> T;
    return T(new EdgePlanner(constraint, planner));
}
