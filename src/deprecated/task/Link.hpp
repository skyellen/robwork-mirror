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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

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
