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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TASK_LINEARJOINTCONSTRAINT_HPP
#define RW_TASK_LINEARJOINTCONSTRAINT_HPP

/**
   @file LinearJointConstraint.hpp
*/

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       A value of type LinearJointConstraint states that the device should move
       from start to goal configuration following a straight line in the
       configuration space.
     */
    class LinearJointConstraint {};

    /**@}*/
}} // end namespaces

#endif // end include guard
