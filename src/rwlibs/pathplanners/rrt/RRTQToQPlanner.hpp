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

#ifndef RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP

/**
   @file RRTQToQPlanner.hpp
*/

#include "RRTTree.hpp"
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/math/Metric.hpp>

#include <vector>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Rapidly Expanding Random Tree based planners for the QToQPlanner
       type of planning problem.

       @relates QToQPlanner
    */
    class RRTQToQPlanner
    {
    public:

        /**
           @brief Basic RRT planner.

           This planner implements BasicPlanner(), page 109 of James J. Kuffner,
           "Autonomous Agensts for Real-Time Animation", 1999.

           The basic version of the planner is a very slow.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        static
        rw::pathplanning::QToQPlannerPtr makeBasic(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

        /**
           @brief RRT-Connect planner.

           Bidirectional RRT planner in the style of James J. Kuffner and Steven
           M. LaValle, "RRT-Connect: An Efficient Approach to Single-Query Path
           Planning", ICRA, 2000.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        static
        rw::pathplanning::QToQPlannerPtr makeConnect(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

        /**
           @brief Bidirectional RRT planner.

           The algorithm of the planner is in the style of
           RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
           "Planning Algorithms", 2006, except this planner is the non-balanced
           version.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        static
        rw::pathplanning::QToQPlannerPtr makeBidirectional(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

        /**
           @brief Balanced, bidirectional RRT planner.

           The algorithm of the planner is in the style of
           RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
           "Planning Algorithms", 2006.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        static
        rw::pathplanning::QToQPlannerPtr makeBalancedBidirectional(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

    private:
        RRTQToQPlanner(const RRTQToQPlanner&);
        RRTQToQPlanner& operator=(const RRTQToQPlanner&);
        RRTQToQPlanner();
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
