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


#ifndef RW_PATHPLANNING_STATECONSTRAINT_HPP
#define RW_PATHPLANNING_STATECONSTRAINT_HPP

/**
   @file StateConstraint.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class StateConstraint;

    //! A pointer to a StateConstraint.
    typedef rw::common::Ptr<StateConstraint> StateConstraintPtr;

    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class StateConstraint
    {
    public:
        /**
           @brief True if the work cell is considered to be in collision for the
           work cell state \b state.
         */
        bool inCollision(const rw::kinematics::State& state) const;

        /**
           Destructor
        */
        virtual ~StateConstraint() {}

        // Factory functions follow below.

        /**
           @brief Map a collision detector to a state constraint.
        */
        static StateConstraintPtr make(
            rw::proximity::CollisionDetectorPtr detector);

        /**
           @brief Combine a set of state constraints into a single state
           constraint.
        */
        static StateConstraintPtr make(
            const std::vector<StateConstraintPtr>& constraints);

    protected:
        /**
           @brief Subclass implementation of the inCollision() method.
        */
        virtual bool doInCollision(const rw::kinematics::State& state) const = 0;

        /**
           @brief Constructor
        */
        StateConstraint() {}

    private:
        StateConstraint(const StateConstraint&);
        StateConstraint& operator=(const StateConstraint&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
