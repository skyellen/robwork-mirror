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

#ifndef RWLIBS_PATHPLANNERS_ARW_ARWPLANNER_HPP
#define RWLIBS_PATHPLANNERS_ARW_ARWPLANNER_HPP

/**
   @file ARWPlanner.hpp
*/

#include "ARWExpand.hpp"
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/models/Device.hpp>
#include <rw/math/Metric.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Adaptive Random Walk planners

       The ARW planners are based on the algorithm of: Stefano Carpin and
       Gianluigi Pillonetto, Motion Planning Using Adaptive Random Walks, IEEE
       Transactions on Robotics, Vol. 21, No. 1, 2005.
    */
    class ARWPlanner
    {
    public:
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
        static
		rw::pathplanning::QToQPlannerPtr
        makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            ARWExpandPtr expand,
            rw::math::QMetricPtr metric,
            double nearDistance);

        /**
           @brief ARW based point-to-point planner.

           Based on \b device, a default distance metric and expansion strategy
           is chosen. A connection to a goal node is attempted if the distance
           is below \b nearDistance. A variance based expansion method is chosen
           with variances being calculated for the latest \b historySize
           samples.

           If \b nearDistance or \b historySize is negative, a default value for
           the parameter is chosen.

           @param constraint [in] Path planning constraint.

           @param device [in] Device for which the path is planned.
        */
        static
		rw::pathplanning::QToQPlannerPtr
        makeQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::models::DevicePtr device,
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
