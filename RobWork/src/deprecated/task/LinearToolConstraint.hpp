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
