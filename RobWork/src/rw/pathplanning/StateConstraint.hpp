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

#include <vector>

namespace rw { namespace common { class Log; } }
namespace rw { namespace proximity { class CollisionDetector; } }
namespace rw { namespace kinematics { class State; } }

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/
    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class StateConstraint
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<StateConstraint> Ptr;
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr< const StateConstraint > CPtr;

		/**
		 * @brief Set the log to be used for writing debug info
		 * @param log [in] Log to which debug information is to be written
		 */
		virtual void setLog(rw::common::Ptr<rw::common::Log> log);

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
		static StateConstraint::Ptr make(
			rw::common::Ptr<rw::proximity::CollisionDetector> detector);

        /**
           @brief Combine a set of state constraints into a single state
           constraint.
        */
		static StateConstraint::Ptr make(
			const std::vector<StateConstraint::Ptr>& constraints);

    protected:
        /**
           @brief Subclass implementation of the inCollision() method.
        */
        virtual bool doInCollision(const rw::kinematics::State& state) const = 0;

        /**
         * @brief Set a log.
         * @param log [in] the log.
         */
		virtual void doSetLog(rw::common::Ptr<rw::common::Log> log) = 0;

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
