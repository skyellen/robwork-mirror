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

#ifndef RW_TASK_LINEARTOOLCONSTRAINT_HPP
#define RW_TASK_LINEARTOOLCONSTRAINT_HPP

/**
   @file LinearToolConstraint.hpp
*/

#include "ToolSpeed.hpp"

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief LinearToolConstraint specifies that the tool in between a pair of
       targets should follow a linear motion.

       A linear motion here means that the position of the tool frame moves
       along a straight line in space, and the orientation of the tool frame
       changes as in slerp interpolation.
    */
    class LinearToolConstraint
    {
    public:
        LinearToolConstraint(const ToolSpeed &tool_speed) :
            _tool_speed(tool_speed)
        {}

    private:
        ToolSpeed _tool_speed;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
