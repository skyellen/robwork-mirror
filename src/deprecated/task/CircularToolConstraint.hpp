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


#ifndef RW_TASK_CIRCULARTOOLCONSTRAINT_HPP
#define RW_TASK_CIRCULARTOOLCONSTRAINT_HPP

/**
   @file CircularToolConstraint.hpp
*/

#include "ToolSpeed.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief CircularToolConstraint specifies that the tool in between a pair
       of targets should follow a circular curve.

       The circle is described by a point relative to a frame through which the
       circular arc should pass.
    */
    class CircularToolConstraint
    {
    public:
        /**
           Constructor
        */
        CircularToolConstraint(
            const ToolSpeed &tool_speed,
            const rw::math::Vector3D<>& via_point,
            rw::kinematics::Frame* via_frame)
            :
            _tool_speed(tool_speed),
            _via_point(via_point),
            _via_frame(via_frame)
        {}

        /**
           The tool speed along the curve.
        */
        const ToolSpeed& getToolSpeed() const { return _tool_speed; }

        /**
           The intermediary point on the curve relative to the frame.
        */
        const rw::math::Vector3D<>& getPoint() const { return _via_point; }

        /**
           The frame relative to which the intermediary point is given.
        */
        rw::kinematics::Frame* getFrame() const { return _via_frame; }

    private:
        ToolSpeed _tool_speed;
        rw::math::Vector3D<> _via_point;
        rw::kinematics::Frame* _via_frame;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
