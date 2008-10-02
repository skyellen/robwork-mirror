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

#ifndef rwlibs_pathplanners_arw_ARWQToQPlanner_HPP
#define rwlibs_pathplanners_arw_ARWQToQPlanner_HPP

/**
   @file ARWQToQPlanner.hpp
*/

#include "ARWExpand.hpp"

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/models/Device.hpp>

#include <vector>
#include <list>
#include <cmath>
#include <climits>

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
            ARWExpandPtr expand,
            rw::math::QMetricPtr metric,
            double nearDistance);

    private:
        bool doQuery(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            rw::trajectory::QPath& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        rw::pathplanning::PlannerConstraint _constraint;
        ARWExpandPtr _expand;
        rw::math::QMetricPtr _metric;
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
