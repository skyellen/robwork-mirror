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


#include "QToQPlanner.hpp"
#include "QToQSamplerPlanner.hpp"
#include "PlannerConstraint.hpp"

using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::trajectory;

namespace
{
    class RegionPlanner : public QToQPlanner
    {
	public:
		RegionPlanner(QToQSamplerPlanner::Ptr planner) :
            _planner(planner)
        {}

    private:
        bool doQuery(
            const Q& from,
            const Q& to,
            QPath& path,
            const StopCriteria& stop)
        {
            return _planner->query(
                from,
                *QSampler::makeSingle(to),
                path,
                stop);
        }

    private:
        QToQSamplerPlanner::Ptr _planner;
    };

    class EdgePlanner : public QToQPlanner
    {
    public:
        EdgePlanner(const PlannerConstraint& constraint)
            : _constraint(constraint)
        {}

    private:
        bool doQuery(
            const Q& from,
            const Q& to,
            QPath& path,
            const StopCriteria& stop)
        {
            if (_constraint.getQConstraint().inCollision(from) ||
                _constraint.getQConstraint().inCollision(to) ||
                _constraint.getQEdgeConstraint().inCollision(from, to))
            {
                return false;
            } else {
                path.push_back(from);
                path.push_back(to);
                return true;
            }
        }

    private:
        PlannerConstraint _constraint;
    };
}

QToQPlanner::Ptr QToQPlanner::make(QToQSamplerPlanner::Ptr planner)
{
    return ownedPtr(new RegionPlanner(planner));
}

QToQPlanner::Ptr QToQPlanner::make(
    const PlannerConstraint& constraint)
{
    return ownedPtr(new EdgePlanner(constraint));
}
