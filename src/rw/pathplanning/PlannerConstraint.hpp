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
