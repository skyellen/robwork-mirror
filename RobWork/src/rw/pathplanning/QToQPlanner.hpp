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


#ifndef RW_PATHPLANNING_QTOQPLANNER_HPP
#define RW_PATHPLANNING_QTOQPLANNER_HPP

/**
   @file QToQPlanner.hpp
*/

#include "PathPlanner.hpp"
#include <rw/common/Ptr.hpp>

namespace rw { namespace pathplanning {
	class PlannerConstraint;
	class QToQSamplerPlanner;

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Path planner interface.

       A path planner plans a path in the configuration space from a start
       configuration to a goal configuration.
    */
    class QToQPlanner : public PathPlanner<rw::math::Q, const rw::math::Q>
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QToQPlanner> Ptr;

        /**
           @brief Construct a path planner from a region planner.

           The region planner is given as goal region the single \b to
           configuration passed to the query() method.

           @param planner [in] A planner for a region given by a QSampler.
        */
		static QToQPlanner::Ptr make(rw::common::Ptr<QToQSamplerPlanner> planner);

        /**
           @brief Construct a path planner from an edge constraint.

           The path planners calls the edge constraint to verify if the path
           going directly from the start to goal configuration can be traversed.

           The configuration constraint is called to verify that neither the
           start nor end configuration is in collision.

           @param constraint [in] Planner constraint.
           @return A planner that attempts the directly connecting edge only.
        */
		static QToQPlanner::Ptr make(const PlannerConstraint& constraint);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
