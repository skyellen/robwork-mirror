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

#ifndef rwlibs_pathplanners_rrt_RRTPlanner_HPP
#define rwlibs_pathplanners_rrt_RRTPlanner_HPP

/**
 * @file RRTPlanner.hpp
 */

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/math/Metric.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include <vector>
#include <list>
#include <cmath>
#include <climits>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief RRT based planners
    */
    class RRTPlanner
    {
    public:
        /**
           @brief RRT based point-to-point planner.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        static rw::pathplanning::QToQPlannerPtr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

        /**
           @brief RRT based point-to-point planner.

           Default configuration space sampling strategy
           (rw::pathplanning::QSampler) and distance metrics (rw:math::QMetric)
           are chosen based on \b device.

           @param constraint [in] Constraint for configurations and edges.
           @param device [in] Device for which the path is planned.
        */
        static rw::pathplanning::QToQPlannerPtr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::models::DevicePtr device);

    private:
        RRTPlanner();
        RRTPlanner(const RRTPlanner&);
        RRTPlanner& operator=(const RRTPlanner&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
