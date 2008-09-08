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

#ifndef rw_pathplanning_QToQPlanner_HPP
#define rw_pathplanning_QToQPlanner_HPP

/**
   @file QToQPlanner.hpp
*/

#include "PathPlanner.hpp"
#include "QToQSamplerPlanner.hpp"
#include "PlannerConstraint.hpp"
#include <rw/common/Ptr.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    class QToQPlanner;

    //! A pointer to a QToQPlanner.
    typedef rw::common::Ptr<QToQPlanner> QToQPlannerPtr;

    /**
       @brief Path planner interface.

       A path planner plans a path in the configuration space from a start
       configuration to a goal configuration.
    */
    class QToQPlanner : public PathPlanner<const rw::math::Q>
    {
    public:
        /**
           @brief Construct a path planner from a region planner.

           The region planner is given as goal region the single \b to
           configuration passed to the query() method.

           @param planner [in] A planner for a region given by a QSampler.
        */
        static QToQPlannerPtr make(QToQSamplerPlannerPtr planner);

        /**
           @brief Construct a path planner from an edge constraint.

           The path planners calls the edge constraint to verify if the path
           going directly from the start to goal configuration can be traversed.

           The configuration constraint is called to verify that neither the
           start nor end configuration is in collision.

           @param constraint [in] Planner constraint.
           @return A planner that attempts the directly connecting edge only.
        */
        static QToQPlannerPtr make(const PlannerConstraint& constraint);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
