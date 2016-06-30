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


#ifndef RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP

/**
   @file RRTQToQPlanner.hpp
*/

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>

namespace rw { namespace pathplanning { class PlannerConstraint; } }
namespace rw { namespace pathplanning { class QSampler; } }

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
        static rw::pathplanning::QToQPlanner::Ptr makeBasic(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::math::QMetric::Ptr metric,
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
			rw::pathplanning::QToQPlanner::Ptr makeConnect(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::math::QMetric::Ptr metric,
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
			rw::pathplanning::QToQPlanner::Ptr makeBidirectional(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::math::QMetric::Ptr metric,
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
			rw::pathplanning::QToQPlanner::Ptr makeBalancedBidirectional(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::math::QMetric::Ptr metric,
            double extend);

    private:
        RRTQToQPlanner(const RRTQToQPlanner&);
        RRTQToQPlanner& operator=(const RRTQToQPlanner&);
        RRTQToQPlanner();
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
