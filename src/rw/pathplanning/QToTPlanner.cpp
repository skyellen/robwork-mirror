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


#include "QToTPlanner.hpp"
#include "QSampler.hpp"

using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::invkin;
using namespace rw::models;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::trajectory;

namespace
{
    class RegionPlanner : public QToTPlanner
    {
    public:
        RegionPlanner(
            QToQSamplerPlannerPtr planner,
            QIKSamplerPtr ikSampler)
            :
            _planner(planner),
            _ikSampler(ikSampler)
        {}

    private:
        bool doQuery(
            const rw::math::Q& from,
            const rw::math::Transform3D<>& baseTend,
            QPath& path,
            const StopCriteria& stop)
        {
            return _planner->query(
                from,
                *QSampler::make(_ikSampler, baseTend),
                path,
                stop);
        }

    private:
        QToQSamplerPlannerPtr _planner;
        QIKSamplerPtr _ikSampler;
    };

    class QToTNearestPlanner : public QToTPlanner
    {
	public:
        QToTNearestPlanner(
            QToQPlannerPtr planner,
            QIKSamplerPtr sampler,
            QMetricPtr metric,
            int cnt)
            :
            _planner(planner),
            _sampler(sampler),
            _metric(metric),
            _cnt(cnt)
        {
            RW_ASSERT(cnt >= 0);
        }

    private:
        bool doQuery(
            const Q& from,
            const Transform3D<>& to,
            QPath& path,
            const StopCriteria& stop)
        {
            Q minQ;
            double minDist = -1;
            for (int cnt = 0; cnt < _cnt && !stop.stop(); cnt++) {
                const Q q = _sampler->sample(to);
                if (!q.empty()) {
                    const double dist = _metric->distance(from, q);
                    if (minDist < 0 || dist < minDist) {
                        minDist = dist;
                        minQ = q;
                    }
                }
            }

            if (minQ.empty()) return false;
            else return _planner->query(from, minQ, path, stop);
        }

    private:
        QToQPlannerPtr _planner;
        QIKSamplerPtr _sampler;
        QMetricPtr _metric;
        int _cnt;
    };
}

QToTPlannerPtr QToTPlanner::make(
    QToQSamplerPlannerPtr planner,
    QIKSamplerPtr ikSampler)
{
    return ownedPtr(new RegionPlanner(planner, ikSampler));
}

QToTPlannerPtr QToTPlanner::makeToNearest(
    QToQPlannerPtr planner,
    QIKSamplerPtr sampler,
    QMetricPtr metric,
    int cnt)
{
    return ownedPtr(new QToTNearestPlanner(planner, sampler, metric, cnt));
}
