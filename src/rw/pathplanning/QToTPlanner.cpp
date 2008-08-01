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

#include "QToTPlanner.hpp"
#include "QSampler.hpp"

using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::invkin;
using namespace rw::models;
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
}

std::auto_ptr<QToTPlanner> QToTPlanner::make(
    QToQSamplerPlannerPtr planner,
    QIKSamplerPtr ikSampler)
{
    typedef std::auto_ptr<QToTPlanner> T;
    return T(new RegionPlanner(planner, ikSampler));
}
