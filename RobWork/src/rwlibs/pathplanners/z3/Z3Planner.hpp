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


#ifndef RWLIBS_PATHPLANNERS_Z3_Z3PLANNER_HPP
#define RWLIBS_PATHPLANNERS_Z3_Z3PLANNER_HPP

/**
   @file Z3Planner.hpp
*/

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>

namespace rw { namespace pathplanning { class PlannerConstraint; } }
namespace rw { namespace pathplanning { class QConstraint; } }
namespace rw { namespace pathplanning { class QSampler; } }
namespace rw { namespace models { class Device; } }

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /* @{ */

    /**
       @brief Z3 based planners

       See "The Z3-Method for Fast Path Planning in Dynamic Environments", Boris
       Baginski, 1996.

       @relates QToQPlanner
    */
    class Z3Planner
    {
    public:
        /**
           @brief Z3 based point-to-point planner.

           @param sampler [in] Sampler of the configuration space.

           @param localPlanner [in] Local planner for connecting the configurations.

           @param nodeCnt [in] Number of supporting configurations to insert.
           If \b nodeCnt is negative, a default value is chosen.

           @param repeatCnt [in] Number of times to repeat the attempt. If \b
           repeatCnt is negative (the default), the attempts are repeated until
           the stop criteria returns true.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::pathplanning::QToQPlanner::Ptr localPlanner,
            int nodeCnt = -1,
            int repeatCnt = -1);

        /**
           @brief Z3 based point-to-point planner.

           A default configuration space sampler (rw::pathplanning::QSampler)
           and local planning is chosen for \b device using \b constraint for
           collision checking.

           @param constraint [in] Constraint for configurations and edges.

           @param device [in] Device for which the path is planned.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::models::Device> device);

        /**
           @brief Sliding local planner.

           This is a variation of the sliding local planner described in the Z3
           paper.

           This is the default local planner used for instantiation of the Z3
           based planners.

           @param constraint [in] Path planning constraint.

           @param directionSampler [in] Sampler of direction vectors in the
           configuration space.

           @param boundsConstraint [in] Constraint checking for the bounds of
           the configuration space.

           @param metric [in] Configuration space distance measure.

           @param extend [in] The length of each sliding step as measured by \b
           metric.

           @param slideImprovement [in] The minimum decrease in distance to the
           goal that should be acheived for every valid slide step. If \b
           slideImprovement is negative, a default value for \b slideImprovement
           is chosen based on the value of \b extend.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeSlidingQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> directionSampler,
			rw::common::Ptr<rw::pathplanning::QConstraint> boundsConstraint,
			rw::math::QMetric::Ptr metric,
            double extend,
            double slideImprovement = -1);

        /**
           @brief Sliding local planner.

           A default direction sampler and bounds checker is chosen for \b
           device.

           @param constraint [in] Path planning constraint.

           @param device [in] Device for which the planning is done.

           @param metric [in] Configuration space distance measure. If no metric
           is given, a default metric for \b device is chosen. In this case \b
           extend and \b slideImprovement should be negative, and default values
           for these will be chosen.

           @param extend [in] The length of each sliding step as measured by \b
           metric.

           @param slideImprovement [in] The minimum decrease in distance to the
           goal that should be acheived for every valid slide step. If \b
           slideImprovement is negative, a default value for \b slideImprovement
           is chosen based on the value of \b extend.
        */
        static rw::pathplanning::QToQPlanner::Ptr makeSlidingQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::models::Device> device,
			rw::math::QMetric::Ptr metric = 0,
            double extend = -1,
            double slideImprovement = -1);

    private:
        Z3Planner();
        Z3Planner(const Z3Planner&);
        Z3Planner& operator=(const Z3Planner&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
