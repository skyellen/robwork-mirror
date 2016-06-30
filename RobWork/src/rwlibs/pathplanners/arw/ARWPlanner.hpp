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


#ifndef RWLIBS_PATHPLANNERS_ARW_ARWPLANNER_HPP
#define RWLIBS_PATHPLANNERS_ARW_ARWPLANNER_HPP

/**
   @file ARWPlanner.hpp
*/

#include "ARWExpand.hpp"
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/math/Metric.hpp>

namespace rw { namespace models { class Device; } }
namespace rw { namespace pathplanning { class PlannerConstraint; } }

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Adaptive Random Walk planners

       The ARW planners are based on the algorithm of: Stefano Carpin and
       Gianluigi Pillonetto, Motion Planning Using Adaptive Random Walks, IEEE
       Transactions on Robotics, Vol. 21, No. 1, 2005.

       @relates QToQPlanner
    */
    class ARWPlanner
    {
    public:
        typedef rw::common::Ptr<ARWPlanner> Ptr;
        /**
           @brief ARW based point-to-point planner.

           The ARW planner expands its paths using instances of \b expand. If
           the end point of one of the paths is within a distance \b
           nearDistance from a goal node when measured with \b metric, then that
           connection is verified. If that connection is valid, the full path is
           returned.

           @param constraint [in] Path planning constraint.

           @param expand [in] ARW expansion strategy.

           @param metric [in] Distance to goal node measure.

           @param nearDistance [in] Threshold for distance to goal node.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			ARWExpand::Ptr expand,
			rw::math::QMetric::Ptr metric,
            double nearDistance);

        /**
           @brief ARW based point-to-point planner.

           Based on \b device, a default distance metric and expansion strategy
           is chosen. A connection to a goal node is attempted if the distance
           is below \b nearDistance. A variance based expansion method is chosen
           with variances being calculated for the latest \b historySize
           samples.

           @param constraint [in] Path planning constraint.

           @param device [in] Device for which the path is planned.

           @param metric [in] Configuration space distance metric. If \b metric
           is NULL, a default metric for \b device is chosen.

           @param nearDistance [in] Try to connect to the goal if the distance
           to the goal measured by \b metric is below this threshold. If \b
           metric is null, a default value for \b nearDistance is chosen.

           @param historySize [in] Number of previous configurations on the path
           to include in computation of the next expand step. If \b historySize
           is negative, a default value for the parameter is chosen.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::models::Device> device,
			rw::math::QMetric::Ptr metric = NULL,
            double nearDistance = -1,
            int historySize = -1);

    private:
        ARWPlanner();
        ARWPlanner(const ARWPlanner&);
        ARWPlanner& operator=(const ARWPlanner&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
