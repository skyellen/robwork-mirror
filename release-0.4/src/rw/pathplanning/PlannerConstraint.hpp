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


#ifndef RW_PATHPLANNING_PLANNERCONSTRAINT_HPP
#define RW_PATHPLANNING_PLANNERCONSTRAINT_HPP

/**
   @file PlannerConstraint.hpp
*/

#include "QConstraint.hpp"
#include "QEdgeConstraint.hpp"

#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class PlannerConstraint;

    //! A pointer to a PlannerConstraint.
    typedef rw::common::Ptr<PlannerConstraint> PlannerConstraintPtr;

    /**
       @brief A tuple of (QConstraintPtr, QEdgeConstraintPtr).

       A planner constraint is a small copyable object containing pointers to a
       configuration constraint and an edge constraint. Sampling based path
       planners and path optimizers typically use a PlannerConstraint object for
       the collision checking for the paths.

       A number of make() utility constructors are provided for applications
       where defaults for configuration and edge constraints can be used.
    */
    class PlannerConstraint
    {
    public:
        /**
           @brief A (QConstraintPtr, QEdgeConstraintPtr) tuple.

           The constraints must be non-null.
        */
        PlannerConstraint(QConstraintPtr constraint, QEdgeConstraintPtr edge);

        /**
           @brief The configuration constraint.
        */
        QConstraint& getQConstraint() const { return *_constraint; }

        /**
           @brief The edge constraint.
        */
        QEdgeConstraint& getQEdgeConstraint() const { return *_edge; }

        /**
           @brief The configuration constraint pointer.
        */
        const QConstraintPtr& getQConstraintPtr() const { return _constraint; }

        /**
           @brief The edge constraint pointer.
        */
        const QEdgeConstraintPtr& getQEdgeConstraintPtr() const { return _edge; }

        /**
           @brief A (QConstraintPtr, QEdgeConstraintPtr) tuple.

           This is equivalent to the standard constructor.
        */
        static PlannerConstraint make(QConstraintPtr constraint, QEdgeConstraintPtr edge);

        /**
           @brief Planner constraint for a collision detector.

           Path are checked discretely for a default device dependent
           resolution.
        */
        static PlannerConstraint make(
            rw::proximity::CollisionDetectorPtr detector,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Planner constraint for a collision strategy.

           Path are checked discretely for a default device dependent
           resolution.

           The default collision setup of the workcell is used.
        */
        static PlannerConstraint make(
            rw::proximity::CollisionStrategyPtr strategy,
            rw::models::WorkCellPtr workcell,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

        /**
           @brief Planner constraint for a collision strategy and collision
           setup.

           Path are checked discretely for a default device dependent
           resolution.
        */
        static PlannerConstraint make(
            rw::proximity::CollisionStrategyPtr strategy,
            const rw::proximity::CollisionSetup& setup,
            rw::models::WorkCellPtr workcell,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state);

    private:
        QConstraintPtr _constraint;
        QEdgeConstraintPtr _edge;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
