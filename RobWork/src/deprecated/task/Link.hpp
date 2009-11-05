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


#ifndef RW_TASK_LINK_HPP
#define RW_TASK_LINK_HPP

/**
   @file Link.hpp
*/

#include "Entity.hpp"
#include "LinearJointConstraint.hpp"
#include "LinearToolConstraint.hpp"
#include "CircularToolConstraint.hpp"

#include <boost/variant.hpp>

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief Link represents a constraint for a motion for a device connecting
       a pair of targets.
    */
    class Link : public Entity
    {
    public:
        //! Variant type for the different forms of constraints.
        typedef boost::variant<
            LinearJointConstraint,
            LinearToolConstraint,
            CircularToolConstraint> Constraint;

        /**
           Constructor
        */
        Link(const Entity& entity, const Constraint& constraint);

        /**
           The constraint for the motion.
        */
        Constraint& getConstraint() { return _value; }

        /**
           The constraint for the motion.
        */
		const Constraint& getConstraint() const { return _value; }

    private:
		Constraint _value;
    };

}} // end namespaces

#endif
