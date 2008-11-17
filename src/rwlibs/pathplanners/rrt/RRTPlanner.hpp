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

#ifndef RWLIBS_PATHPLANNERS_RRT_RRTPLANNER_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTPLANNER_HPP

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

       @relates QToQPlanner
    */
    class RRTPlanner
    {
    public:
        //! The type of RRT planner to construct.
        enum PlannerType {
            /**
               @brief Simple non-greedy, bidirectional RRT.

               See BasicPlanner(), page 109 of James J. Kuffner, "Autonomous
               Agensts for Real-Time Animation", 1999.
            */
            RRTBasic,

            /**
               @brief RRT-Connect planner.

               See James J. Kuffner and Steven M. LaValle, "RRT-Connect: An
               Efficient Approach to Single-Query Path Planning", ICRA, 2000.
            */
            RRTConnect,

            /**
               @brief Bidirectional RRT.

               The algorithm of the planner is in the style of
               RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
               "Planning Algorithms", 2006, except this planner is the non-balanced
               version.
            */
            RRTBidirectional,

            /**
               @brief Balanced, bidirectional RRT.

               The algorithm of the planner is in the style of
               RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
               "Planning Algorithms", 2006.
            */
            RRTBalancedBidirectional
        };

        /**
           @brief RRT based point-to-point planner.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.

           @param type [in] The particular variation the RRT planner algorithm.
        */
        static rw::pathplanning::QToQPlannerPtr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend,
            PlannerType type = RRTBalancedBidirectional);

        /**
           @brief RRT based point-to-point planner.

           Default configuration space sampling strategy
           (rw::pathplanning::QSampler) and distance metrics (rw:math::QMetric)
           are chosen based on \b device.

           @param constraint [in] Constraint for configurations and edges.

           @param device [in] Device for which the path is planned.

           @param type [in] The particular variation the RRT planner algorithm.
        */
        static rw::pathplanning::QToQPlannerPtr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::models::DevicePtr device,
            PlannerType type = RRTBalancedBidirectional);

    private:
        RRTPlanner();
        RRTPlanner(const RRTPlanner&);
        RRTPlanner& operator=(const RRTPlanner&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
