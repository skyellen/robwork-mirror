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

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class WorkCell; } }
namespace rw { namespace proximity { class CollisionDetector; } }
namespace rw { namespace proximity { class CollisionSetup; } }
namespace rw { namespace proximity { class CollisionStrategy; } }

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

#ifdef RW_USE_DEPRECATED
    class PlannerConstraint;

    //! Deprecated: A pointer to a PlannerConstraint.
    typedef rw::common::Ptr<PlannerConstraint> PlannerConstraintPtr;
#endif 

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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<PlannerConstraint> Ptr;

		/**
		 * @brief Default constructed without constraints initialized
		 */
		PlannerConstraint();

        /**
           @brief A (QConstraintPtr, QEdgeConstraintPtr) tuple.

           The constraints must be non-null.
        */
		PlannerConstraint(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

		/**
		 * @brief Forwards call to the QConstraint wrapped by the PlannerConstraint
		 */
		bool inCollision(const rw::math::Q& q);

		/**
		 * @brief Forwards call to the QEdgeConstraint wrapped by the PlannerConstraint
		 */
		bool inCollision(const rw::math::Q& q1, const rw::math::Q& q2);

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
		const QConstraint::Ptr& getQConstraintPtr() const { return _constraint; }

        /**
           @brief The edge constraint pointer.
        */
		const QEdgeConstraint::Ptr& getQEdgeConstraintPtr() const { return _edge; }

        /**
           @brief A (QConstraintPtr, QEdgeConstraintPtr) tuple.

           This is equivalent to the standard constructor.
        */
		static PlannerConstraint make(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

        /**
           @brief Planner constraint for a collision detector.

           Path are checked discretely for a default device dependent
           resolution.
        */
		static PlannerConstraint make(rw::common::Ptr<rw::proximity::CollisionDetector> detector,
				rw::common::Ptr<rw::models::Device> device,
				const rw::kinematics::State& state);

		/**
           @brief Planner constraint for a collision strategy.

           Path are checked discretely for a default device dependent
           resolution.

           The default collision setup of the workcell is used.
        */
		static PlannerConstraint make(rw::common::Ptr<rw::proximity::CollisionStrategy> strategy,
			rw::common::Ptr<rw::models::WorkCell> workcell,
			rw::common::Ptr<rw::models::Device> device,
			const rw::kinematics::State& state);

        /**
           @brief Planner constraint for a collision strategy and collision
           setup.

           Path are checked discretely for a default device dependent
           resolution.
        */
		static PlannerConstraint make(rw::common::Ptr<rw::proximity::CollisionStrategy> strategy,
			const rw::proximity::CollisionSetup& setup,
			rw::common::Ptr<rw::models::WorkCell> workcell,
			rw::common::Ptr<rw::models::Device> device,
			const rw::kinematics::State& state);

    private:
		QConstraint::Ptr _constraint;
		QEdgeConstraint::Ptr _edge;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
