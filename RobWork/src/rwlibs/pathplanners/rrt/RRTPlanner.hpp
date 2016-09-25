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


#ifndef RWLIBS_PATHPLANNERS_RRT_RRTPLANNER_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTPLANNER_HPP

/**
 * @file RRTPlanner.hpp
 */

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>

namespace rw { namespace pathplanning { class PlannerConstraint; } }
namespace rw { namespace pathplanning { class QSampler; } }
namespace rw { namespace models { class Device; } }

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
    	//! @brief Smart pointer type for a RRTPlanner.
        typedef rw::common::Ptr<RRTPlanner> Ptr;

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
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::math::QMetric::Ptr metric,
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
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::models::Device> device,
            PlannerType type = RRTBalancedBidirectional);

    private:
        RRTPlanner();
        RRTPlanner(const RRTPlanner&);
        RRTPlanner& operator=(const RRTPlanner&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
