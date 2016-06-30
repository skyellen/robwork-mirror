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


#ifndef RWLIBS_PATHPLANNERS_ARW_ARWQTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_ARW_ARWQTOQPLANNER_HPP

/**
   @file ARWQToQPlanner.hpp
*/

#include "ARWExpand.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>

#include <vector>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Adaptive Random Walk Planner

       ARWQToQPlanner is an implementation of the planner of: Stefano Carpin and
       Gianluigi Pillonetto, Motion Planning Using Adaptive Random Walks, IEEE
       Transactions on Robotics, Vol. 21, No. 1, 2005.
    */
    class ARWQToQPlanner : public rw::pathplanning::QToQPlanner
    {
    public:
        /**
           @brief Constructor

           \b nearDistance must be non-negative.

           @param constraint [in] Path planning constraint.
           @param expand [in] Expansion strategy for the random walk.
           @param metric [in] Distance metric on the configuration space.

           @param nearDistance [in] Attempt connection to goal node if the
           distance to the node is less than \b nearDistance when measured by \b
           metric.
        */
        ARWQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			ARWExpand::Ptr expand,
			rw::math::QMetric::Ptr metric,
            double nearDistance);

    private:
        bool doQuery(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            rw::trajectory::QPath& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        rw::pathplanning::PlannerConstraint _constraint;
		ARWExpand::Ptr _expand;
		rw::math::QMetric::Ptr _metric;
        double _nearDistance;

    private:
        bool nearGoal(const rw::math::Q& q, const rw::math::Q& goal) const;
        bool planPathStep(
            ARWExpand& expand,
            const std::vector<rw::math::Q>& goals) const;
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
